#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# Matplotlib for GUI (only if enabled)
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


def wrap_to_pi(angle: float) -> float:
    """Wrap any angle into [−π, +π]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a


class SwerveDriveController(Node):
    """
    ROS2 Humble Python node to control a 4‐wheel swerve platform, with optional Matplotlib GUI.

    • `enable_gui=True` will launch a 2×2 plot of circles showing each module’s arrows.
    • Horizontal translation (vx, vy) is handled directly by swerve kinematics.
    • Pure rotation (vx≈0, vy≈0, ω≠0) comes “for free” from the ±90° optimization logic—no
      additional angle offsets are needed.
    • After computing raw angles via atan2(Vy, Vx), we constrain each wheel’s steering to
      [−π/2, +π/2] by flipping angle ±π and inverting speed as necessary.
    """

    def __init__(self):
        super().__init__('swerve_drive_controller')

        # === PARAMETERS ===
        self.declare_parameter('wheelbase', 0.5)      # Distance front-to-rear (L) [m]
        self.declare_parameter('track_width', 0.5)    # Distance left-to-right (W) [m]
        self.declare_parameter('steering_tolerance_deg', 2.0)
        self.declare_parameter('enable_gui', True)

        # Standard module names
        self.modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
        self.steer_joint_names = {m: f'{m}_steer' for m in self.modules}
        self.drive_joint_names = {m: f'{m}_drive' for m in self.modules}
        self.steer_cmd_topics = {m: f'/{m}_steer_controller/commands' for m in self.modules}
        self.drive_cmd_topics = {m: f'/{m}_drive_controller/commands' for m in self.modules}

        # Pull and store parameters
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.W = self.get_parameter('track_width').get_parameter_value().double_value
        tol_deg = self.get_parameter('steering_tolerance_deg').get_parameter_value().double_value
        self.steering_tolerance = math.radians(tol_deg)
        self.enable_gui = self.get_parameter('enable_gui').get_parameter_value().bool_value

        # Precompute each module’s (x_i, y_i) relative to robot center: x forward, y left
        half_L = self.L / 2.0
        half_W = self.W / 2.0
        self.module_positions = {
            'front_left':  ( half_L,  half_W),
            'front_right': ( half_L, -half_W),
            'rear_left':   (-half_L,  half_W),
            'rear_right':  (-half_L, -half_W),
        }

        # === STATE ===
        # Current (actual) steering angles & drive velocities (from /joint_states)
        self.current_steering_angles = {m: None for m in self.modules}
        self.current_drive_velocities = {m: None for m in self.modules}

        # Target (desired) steering angles & drive speeds (set on each /cmd_vel)
        self.target_steering_angles = {m: None for m in self.modules}
        self.target_drive_speeds = {m: None for m in self.modules}

        # “Ready” flags per module, and global all_steer_ready
        self.steer_ready = {m: False for m in self.modules}
        self.all_steer_ready = False

        # === PUBLISHERS & SUBSCRIBERS ===
        self.steer_pubs = {}
        self.drive_pubs = {}
        for m in self.modules:
            self.steer_pubs[m] = self.create_publisher(
                Float64MultiArray,
                self.steer_cmd_topics[m],
                10,
            )
            self.drive_pubs[m] = self.create_publisher(
                Float64MultiArray,
                self.drive_cmd_topics[m],
                10,
            )

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Timer (~10 Hz) to check if all steers reached their target before sending drives
        self.create_timer(0.1, self.check_all_steer_and_publish_drives)

        # === GUI SETUP (if enabled) ===
        if self.enable_gui:
            self.init_gui()
            # Run ROS spin in background thread so GUI can update in main thread
            ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
            ros_thread.start()
            self.get_logger().info('SwerveDriveController initialized—GUI enabled.')
            self.run_gui()
        else:
            self.get_logger().info('SwerveDriveController initialized—running headless.')
            rclpy.spin(self)

    def _spin_ros(self):
        """Background rclpy.spin() for GUI mode."""
        rclpy.spin(self)

    # ---------- GUI METHODS ----------
    def init_gui(self):
        """Initialize a 2×2 Matplotlib window with arrows & velocity texts."""
        plt.ion()
        self.fig, axs = plt.subplots(2, 2, figsize=(8, 8))
        axs = axs.flatten()

        self.quiver_current = {}
        self.quiver_desired = {}
        self.text_current = {}
        self.text_target = {}

        for idx, m in enumerate(self.modules):
            ax = axs[idx]
            ax.set_aspect('equal')
            ax.set_xlim(-1.2, 1.2)
            ax.set_ylim(-1.2, 1.2)

            circle = Circle((0, 0), radius=1.0, fill=False, color='black', linewidth=1.5)
            ax.add_patch(circle)

            # Initialize zero‐length arrows at center
            q_c = ax.quiver(0, 0, 0, 0,
                            angles='xy', scale_units='xy', scale=1,
                            color='blue', width=0.02)
            q_d = ax.quiver(0, 0, 0, 0,
                            angles='xy', scale_units='xy', scale=1,
                            color='green', width=0.02)
            self.quiver_current[m] = q_c
            self.quiver_desired[m] = q_d

            # Place text for current (blue) and desired (green) velocities
            txt_c = ax.text(-1.1, 1.05, "0.00", color='blue', fontsize=10, weight='bold')
            txt_d = ax.text(-1.1, 0.85, "0.00", color='green', fontsize=10, weight='bold')
            self.text_current[m] = txt_c
            self.text_target[m] = txt_d

            ax.set_title(m.replace('_', ' ').title())
            ax.axis('off')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run_gui(self):
        """
        Loop ~10 Hz to update arrows and texts in the Matplotlib window.
        Blocks until the window is closed.
        """
        rate = self.create_rate(10)
        try:
            while rclpy.ok():
                self.update_gui()
                plt.pause(0.001)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            plt.close(self.fig)

    def update_gui(self):
        """Called in the GUI loop to refresh arrow directions and velocity labels."""
        for m in self.modules:
            curr_ang = self.current_steering_angles.get(m) or 0.0
            tar_ang = self.target_steering_angles.get(m) or 0.0

            curr_vel = self.current_drive_velocities.get(m) or 0.0
            tar_vel = self.target_drive_speeds.get(m) or 0.0

            # Normalize speeds to [0, 4] → [0, 1]
            norm_c = min(abs(curr_vel), 4.0) / 4.0
            norm_d = min(abs(tar_vel), 4.0) / 4.0

            # Scale arrow length 30%–100% of 0.9 radius
            scale_c = 0.3 + 0.7 * norm_c
            scale_d = 0.3 + 0.7 * norm_d
            length_c = scale_c * 0.9
            length_d = scale_d * 0.9

            # If velocity < 0, flip arrow 180°
            disp_curr_ang = curr_ang + (math.pi if curr_vel < 0 else 0.0)
            disp_tar_ang = tar_ang + (math.pi if tar_vel < 0 else 0.0)

            # Map: 0° → up; +90° → left; 180° → down; –90° → right
            u_c = -math.sin(disp_curr_ang) * length_c
            v_c =  math.cos(disp_curr_ang) * length_c
            u_d = -math.sin(disp_tar_ang) * length_d
            v_d =  math.cos(disp_tar_ang) * length_d

            self.quiver_current[m].set_UVC(u_c, v_c)
            self.quiver_desired[m].set_UVC(u_d, v_d)

            self.text_current[m].set_text(f"{curr_vel:.2f}")
            self.text_target[m].set_text(f"{tar_vel:.2f}")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # ---------- CONTROL METHODS ----------
    def cmd_vel_callback(self, msg: Twist):
        """
        On each /cmd_vel:
        1) Compute raw (angle_i, speed_i) from swerve kinematics:
             Vx_i = vx – ω·y_i
             Vy_i = vy + ω·x_i
             angle_i = atan2(Vy_i, Vx_i)
             speed_i = sqrt(Vx_i² + Vy_i²)
        2) Constrain angle_i into [–π/2, +π/2] by ±π flip & speed inversion.
        3) If all four steers already within tolerance of their new angles, publish drives immediately.
        4) Otherwise: stop all drives, send new steering targets, and wait for the timer to release drives.
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # 1) Raw kinematic angles & speeds
        raw_angles = {}
        raw_speeds = {}
        for m, (x_i, y_i) in self.module_positions.items():
            Vx_i = vx - omega * y_i
            Vy_i = vy + omega * x_i
            speed_i = math.hypot(Vx_i, Vy_i)
            angle_i = math.atan2(Vy_i, Vx_i)
            raw_angles[m] = angle_i
            raw_speeds[m] = speed_i

        # 2) ±90° optimization
        new_angles = {}
        new_speeds = {}
        for m in self.modules:
            ang = wrap_to_pi(raw_angles[m])
            spd = raw_speeds[m]
            if ang > math.pi / 2:
                ang -= math.pi
                spd = -spd
            elif ang < -math.pi / 2:
                ang += math.pi
                spd = -spd
            new_angles[m] = wrap_to_pi(ang)
            new_speeds[m] = spd

        # 3) If already “all_steer_ready” AND all current angles within tolerance, publish drives
        if self.all_steer_ready:
            all_within = True
            for m in self.modules:
                curr_ang = self.current_steering_angles.get(m)
                if curr_ang is None:
                    all_within = False
                    break
                if abs(self._angle_diff(new_angles[m], curr_ang)) > self.steering_tolerance:
                    all_within = False
                    break

            if all_within:
                for m in self.modules:
                    self.target_drive_speeds[m] = new_speeds[m]
                    drive_msg = Float64MultiArray()
                    drive_msg.data = [self.target_drive_speeds[m]]
                    self.drive_pubs[m].publish(drive_msg)
                self.get_logger().info('All steers within tolerance → published updated drive speeds.')
                for m in self.modules:
                    self.target_steering_angles[m] = new_angles[m]
                return

        # 4) Otherwise, halt all drives, update steering targets, then wait for check timer
        for m in self.modules:
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0]
            self.drive_pubs[m].publish(stop_msg)
        self.get_logger().info('Stopping all drives before re-steering.')

        for m in self.modules:
            self.target_steering_angles[m] = new_angles[m]
            self.target_drive_speeds[m] = new_speeds[m]
            self.steer_ready[m] = False
        self.all_steer_ready = False

        for m in self.modules:
            steer_msg = Float64MultiArray()
            steer_msg.data = [self.target_steering_angles[m]]
            self.steer_pubs[m].publish(steer_msg)
        self.get_logger().info('Published optimized steering targets (±90° constraint).')

    def joint_states_callback(self, msg: JointState):
        """Update current steering angles & drive velocities from /joint_states."""
        for i, name in enumerate(msg.name):
            for m in self.modules:
                if name == self.steer_joint_names[m]:
                    self.current_steering_angles[m] = msg.position[i]
                if name == self.drive_joint_names[m]:
                    self.current_drive_velocities[m] = msg.velocity[i]

    def check_all_steer_and_publish_drives(self):
        """
        Called ~10 Hz: if all steering angles are within ±tolerance, publish all drive speeds.
        """
        if any(self.target_steering_angles[m] is None for m in self.modules):
            return
        if self.all_steer_ready:
            return
        if any(self.current_steering_angles[m] is None for m in self.modules):
            return

        all_now_ready = True
        for m in self.modules:
            err = abs(self._angle_diff(
                self.target_steering_angles[m],
                self.current_steering_angles[m]
            ))
            if err <= self.steering_tolerance:
                self.steer_ready[m] = True
            else:
                self.steer_ready[m] = False
                all_now_ready = False

        if all_now_ready:
            self.all_steer_ready = True
            for m in self.modules:
                drive_msg = Float64MultiArray()
                drive_msg.data = [self.target_drive_speeds[m]]
                self.drive_pubs[m].publish(drive_msg)
            self.get_logger().info('All steers within tolerance → published drive speeds.')

    @staticmethod
    def _angle_diff(goal: float, current: float) -> float:
        """Wrapped difference between two angles (in [−π, +π])."""
        a = (goal - current) % (2.0 * math.pi)
        if a > math.pi:
            a -= 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveController()
    # If GUI disabled, spin was called in __init__. Otherwise, GUI loop is running.
    try:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down SwerveDriveController...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# import math

# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Twist
# from std_msgs.msg import Float64MultiArray


# class SwerveDriveController(Node):
#     """
#     ROS2 Humble Python node to control a 4‐wheel swerve platform.
#     - Subscribes to /cmd_vel (geometry_msgs/Twist) and /joint_states (sensor_msgs/JointState).
#     - Publishes steering‐position commands and drive‐velocity commands (both as Float64MultiArray)
#       for each of the four modules: front_left, front_right, rear_left, rear_right.
#     - On every new cmd_vel:
#         • Compute per‐wheel target angles & speeds via standard swerve kinematics.
#         • If all wheels are already within ±tolerance of their new angles (and were “ready”),
#           immediately send the new drive speeds.
#         • Otherwise, stop all drives (pub 0.0), send the four steering setpoints, and wait.
#       A 10 Hz timer checks when all four steers are within tolerance—and only then sends the
#       four drive speeds once.

#     Default joint & topic names assume the typical ros2_control naming:
#       • Steering joints:   front_left_steer, front_right_steer, rear_left_steer, rear_right_steer
#       • Drive joints:      front_left_drive, front_right_drive, rear_left_drive, rear_right_drive
#       • Steering cmd topics: /<module>_steer_controller/commands
#       • Drive cmd topics:    /<module>_drive_controller/commands

#     Adjust parameters or hardcode your actual names as needed.
#     """

#     def __init__(self):
#         super().__init__('swerve_drive_controller')

#         # === PARAMETERS ===
#         # Robot geometry (meters)
#         self.declare_parameter('wheelbase', 1)      # Distance between front & rear wheels (L)
#         self.declare_parameter('track_width', 1)    # Distance between left & right wheels (W)
#         # Steering tolerance in degrees
#         self.declare_parameter('steering_tolerance_deg', 2.0)

#         # Joint‐and‐topic naming conventions per module
#         modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
#         self.modules = modules

#         # For each module, we assume joints:
#         #   <module>_steer  and  <module>_drive
#         # Topics:
#         #   /<module>_steer_controller/commands
#         #   /<module>_drive_controller/commands
#         # If your actual names differ, update these dicts or pass parameters instead.
#         self.steer_joint_names = {
#             mod: f'{mod}_steer'
#             for mod in modules
#         }
#         self.drive_joint_names = {
#             mod: f'{mod}_drive'
#             for mod in modules
#         }
#         self.steer_cmd_topics = {
#             mod: f'/{mod}_steer_controller/commands'
#             for mod in modules
#         }
#         self.drive_cmd_topics = {
#             mod: f'/{mod}_drive_controller/commands'
#             for mod in modules
#         }

#         # Pull parameter values
#         self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
#         self.W = self.get_parameter('track_width').get_parameter_value().double_value
#         self.steering_tolerance = math.radians(
#             self.get_parameter('steering_tolerance_deg').get_parameter_value().double_value
#         )

#         # Precompute module positions relative to robot center (x forward, y left)
#         # front_left  = (+L/2, +W/2)
#         # front_right = (+L/2, −W/2)
#         # rear_left   = (−L/2, +W/2)
#         # rear_right  = (−L/2, −W/2)
#         half_L = self.L / 2.0
#         half_W = self.W / 2.0
#         self.module_positions = {
#             'front_left':  ( half_L,  half_W),
#             'front_right': ( half_L, -half_W),
#             'rear_left':   (-half_L,  half_W),
#             'rear_right':  (-half_L, -half_W),
#         }

#         # === INTERNAL STATE ===
#         # Current steering angles & drive velocities (from joint_states)
#         self.current_steering_angles = {mod: None for mod in modules}
#         self.current_drive_velocities = {mod: None for mod in modules}

#         # Target angles & speeds (set on each new cmd_vel)
#         self.target_steering_angles = {mod: None for mod in modules}
#         self.target_drive_speeds = {mod: None for mod in modules}

#         # Per‐module “ready” flags, plus a global “all_ready”
#         self.steer_ready = {mod: False for mod in modules}
#         self.all_steer_ready = False

#         # === PUBLISHERS ===
#         self.steer_pubs = {}
#         self.drive_pubs = {}
#         for mod in modules:
#             self.steer_pubs[mod] = self.create_publisher(
#                 Float64MultiArray,
#                 self.steer_cmd_topics[mod],
#                 10
#             )
#             self.drive_pubs[mod] = self.create_publisher(
#                 Float64MultiArray,
#                 self.drive_cmd_topics[mod],
#                 10
#             )

#         # === SUBSCRIBERS ===
#         self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_vel_callback,
#             10
#         )
#         self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_states_callback,
#             10
#         )

#         # Timer to check steering readiness ~10 Hz
#         self.create_timer(0.1, self.check_all_steer_and_publish_drives)

#         self.get_logger().info('SwerveDriveController initialized. Waiting for /cmd_vel and /joint_states…')

#     def cmd_vel_callback(self, msg: Twist):
#         """
#         On new /cmd_vel:
#           1) Compute per‐module (angle, speed) via swerve kinematics.
#           2) If all four current steering angles are within ±tolerance of the new targets
#              (and previously marked “all_ready”), publish new drive speeds immediately.
#           3) Otherwise:
#              a) Publish zero to all four drives (stop wheels).
#              b) Update all target angles & speeds, reset flags, publish steering setpoints.
#              c) Defer publishing drive speeds until all steers are ready (checked in a timer).
#         """

#         vx = msg.linear.x
#         vy = msg.linear.y
#         omega = msg.angular.z  # yaw rate

#         # 1) Compute desired (angle, speed) for each module using standard swerve formula
#         # For each module at (x_i, y_i):
#         #   Vx_i = vx − omega * y_i
#         #   Vy_i = vy + omega * x_i
#         #   speed_i = sqrt(Vx_i² + Vy_i²)
#         #   angle_i = atan2(Vy_i, Vx_i)
#         new_angles = {}
#         new_speeds = {}
#         for mod, (x_i, y_i) in self.module_positions.items():
#             Vx_i = vx - omega * y_i
#             Vy_i = vy + omega * x_i
#             speed_i = math.hypot(Vx_i, Vy_i)
#             angle_i = math.atan2(Vy_i, Vx_i)
#             new_angles[mod] = angle_i
#             new_speeds[mod] = speed_i

#         # 2) Check if “all_ready” and current angles are within tolerance of new_angles
#         already_ready = self.all_steer_ready
#         if already_ready:
#             # Verify each module’s current steering vs. new target
#             all_within = True
#             for mod in self.modules:
#                 curr_ang = self.current_steering_angles[mod]
#                 if curr_ang is None:
#                     all_within = False
#                     break
#                 err = abs(self._angle_diff(new_angles[mod], curr_ang))
#                 if err > self.steering_tolerance:
#                     all_within = False
#                     break

#             if all_within:
#                 # Immediately publish the new drive speeds, update targets, and return
#                 for mod in self.modules:
#                     self.target_drive_speeds[mod] = new_speeds[mod]
#                     drive_msg = Float64MultiArray()
#                     drive_msg.data = [self.target_drive_speeds[mod]]
#                     self.drive_pubs[mod].publish(drive_msg)
#                 self.get_logger().info('All steers already within tolerance → published new drive speeds.')
#                 # Update angles too, in case the next cmd_vel also has same angles
#                 for mod in self.modules:
#                     self.target_steering_angles[mod] = new_angles[mod]
#                 return

#         # 3) Otherwise, we must re‐steer:
#         # 3a) Stop all drives immediately
#         for mod in self.modules:
#             stop_msg = Float64MultiArray()
#             stop_msg.data = [0.0]
#             self.drive_pubs[mod].publish(stop_msg)
#         self.get_logger().info('Published 0.0 to all 4 drives to stop wheels before re‐steering.')

#         # 3b) Update targets, reset flags
#         for mod in self.modules:
#             self.target_steering_angles[mod] = new_angles[mod]
#             self.target_drive_speeds[mod] = new_speeds[mod]
#             self.steer_ready[mod] = False
#         self.all_steer_ready = False

#         # 3c) Publish new steering setpoints for all four modules
#         for mod in self.modules:
#             steer_msg = Float64MultiArray()
#             steer_msg.data = [self.target_steering_angles[mod]]
#             self.steer_pubs[mod].publish(steer_msg)
#         self.get_logger().info('Published new steering targets for all four modules.')

#         # 3d) Defer drive speeds until `check_all_steer_and_publish_drives` sees all wheels within tolerance.

#     def joint_states_callback(self, msg: JointState):
#         """
#         Update current steering angles & drive velocities from /joint_states.
#         Expects msg.name[] to contain each of the eight joints (4 steer + 4 drive).
#         """
#         for i, name in enumerate(msg.name):
#             # Steering joints
#             for mod in self.modules:
#                 if name == self.steer_joint_names[mod]:
#                     self.current_steering_angles[mod] = msg.position[i]
#             # Drive joints (we track velocity if you need it, but not used for control logic)
#             for mod in self.modules:
#                 if name == self.drive_joint_names[mod]:
#                     self.current_drive_velocities[mod] = msg.velocity[i]

#     def check_all_steer_and_publish_drives(self):
#         """
#         Called at 10 Hz: checks if each of the 4 steering angles is within tolerance.
#         If so, and if we haven’t yet published the drive speeds for this target, do so now.
#         """
#         # If we never received a target, skip
#         if any(self.target_steering_angles[mod] is None for mod in self.modules):
#             return

#         # If we already published drive speeds for this steering target, skip
#         if self.all_steer_ready:
#             return

#         # If we don’t have current info for all modules yet, skip
#         if any(self.current_steering_angles[mod] is None for mod in self.modules):
#             return

#         # Check each module’s angle error
#         all_now_ready = True
#         for mod in self.modules:
#             err = abs(self._angle_diff(
#                 self.target_steering_angles[mod],
#                 self.current_steering_angles[mod]
#             ))
#             if err <= self.steering_tolerance:
#                 self.steer_ready[mod] = True
#             else:
#                 self.steer_ready[mod] = False
#                 all_now_ready = False

#         if all_now_ready:
#             # Mark global flag
#             self.all_steer_ready = True
#             # Publish all four drive speeds
#             for mod in self.modules:
#                 drive_msg = Float64MultiArray()
#                 drive_msg.data = [self.target_drive_speeds[mod]]
#                 self.drive_pubs[mod].publish(drive_msg)
#             self.get_logger().info('All steers within ±tolerance → published all 4 drive speeds.')

#     @staticmethod
#     def _angle_diff(goal: float, current: float) -> float:
#         """
#         Shortest signed difference between two angles (radians), wrapped to [−π, +π].
#         """
#         a = (goal - current) % (2.0 * math.pi)
#         if a > math.pi:
#             a -= 2.0 * math.pi
#         return a


# def main(args=None):
#     rclpy.init(args=args)
#     node = SwerveDriveController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.get_logger().info('Shutting down SwerveDriveController...')
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
