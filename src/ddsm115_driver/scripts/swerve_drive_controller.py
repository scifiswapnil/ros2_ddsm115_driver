#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# Matplotlib for GUI
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


class SwerveDriveController(Node):
    """
    ROS2 Humble Python node to control a 4‐wheel swerve platform, with a simple Matplotlib GUI
    displaying each module’s current vs. desired angle (blue vs. green arrows) and velocities.

    Subscriptions:
      • /cmd_vel           (geometry_msgs/Twist)
      • /joint_states      (sensor_msgs/JointState)

    Publications (per module):
      • /<module>_steer_controller/commands   (std_msgs/Float64MultiArray)  → steering angle [rad]
      • /<module>_drive_controller/commands   (std_msgs/Float64MultiArray)  → drive speed [m/s]

    GUI: A 2×2 grid of circles, one per module (front_left, front_right, rear_left, rear_right).
      • Blue arrow: current steering angle (length ∝ |current_speed|, scaled 30%–100% of max).
                    If current_speed < 0, arrow points 180° opposite the steering angle.
      • Green arrow: desired steering angle (length ∝ |desired_speed|, scaled 30%–100% of max).
                     If desired_speed < 0, arrow points 180° opposite the steering angle.
      • Blue text (top-left): current drive velocity (m/s).
      • Green text (just below blue): desired drive velocity (m/s).

    Steering optimization: After computing raw angle_i via atan2, if |angle_i| > 90° (π/2),
    adjust angle_i by ±π to bring it into [−π/2, +π/2] and flip speed_i sign. Thus GUI arrows
    and published commands always use angles within ±90°.
    """

    def __init__(self):
        super().__init__('swerve_drive_controller')

        # === PARAMETERS ===
        self.declare_parameter('wheelbase', 0.5)      # Distance front-to-rear (L) [m]
        self.declare_parameter('track_width', 0.4)    # Distance left-to-right (W) [m]
        self.declare_parameter('steering_tolerance_deg', 2.0)  # ± tolerance in degrees

        # Module identifiers
        self.modules = ['front_left', 'front_right', 'rear_left', 'rear_right']

        # Joint names assumed per module
        self.steer_joint_names = {
            mod: f'{mod}_steer'
            for mod in self.modules
        }
        self.drive_joint_names = {
            mod: f'{mod}_drive'
            for mod in self.modules
        }

        # Command topics assumed per module
        self.steer_cmd_topics = {
            mod: f'/{mod}_steer_controller/commands'
            for mod in self.modules
        }
        self.drive_cmd_topics = {
            mod: f'/{mod}_drive_controller/commands'
            for mod in self.modules
        }

        # Pull parameter values
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.W = self.get_parameter('track_width').get_parameter_value().double_value
        tol_deg = self.get_parameter('steering_tolerance_deg').get_parameter_value().double_value
        self.steering_tolerance = math.radians(tol_deg)

        # Precompute each module’s (x_i, y_i) relative to robot center (x forward, y left)
        half_L = self.L / 2.0
        half_W = self.W / 2.0
        self.module_positions = {
            'front_left':  ( half_L,  half_W),
            'front_right': ( half_L, -half_W),
            'rear_left':   (-half_L,  half_W),
            'rear_right':  (-half_L, -half_W),
        }

        # === INTERNAL STATE ===
        # Current steering angles & drive velocities (updated from /joint_states)
        self.current_steering_angles = {mod: None for mod in self.modules}
        self.current_drive_velocities = {mod: None for mod in self.modules}

        # Target (desired) steering angles & drive speeds (set each /cmd_vel)
        self.target_steering_angles = {mod: None for mod in self.modules}
        self.target_drive_speeds = {mod: None for mod in self.modules}

        # Per-module “steer_ready” flags, plus a global "all_steer_ready"
        self.steer_ready = {mod: False for mod in self.modules}
        self.all_steer_ready = False

        # === PUBLISHERS ===
        self.steer_pubs = {}
        self.drive_pubs = {}
        for mod in self.modules:
            self.steer_pubs[mod] = self.create_publisher(
                Float64MultiArray,
                self.steer_cmd_topics[mod],
                10
            )
            self.drive_pubs[mod] = self.create_publisher(
                Float64MultiArray,
                self.drive_cmd_topics[mod],
                10
            )

        # === SUBSCRIBERS ===
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # Timer to check if all steering axes are within tolerance (~10 Hz)
        self.create_timer(0.1, self.check_all_steer_and_publish_drives)

        # === GUI SETUP ===
        # We run the Matplotlib event loop in the main thread, and let ROS spin in a background thread.
        self.init_gui()

        # Spin ROS in a background thread
        ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        ros_thread.start()

        self.get_logger().info('SwerveDriveController initialized—GUI and ROS spinning.')

        # Start the Matplotlib interactive loop (blocks until window closed)
        self.run_gui()

    def _spin_ros(self):
        """Runs rclpy.spin in a background thread."""
        rclpy.spin(self)

    def init_gui(self):
        """Initializes the Matplotlib figure, axes (2×2), and arrow/text artists."""
        plt.ion()  # Turn on interactive mode
        self.fig, axs = plt.subplots(2, 2, figsize=(8, 8))
        axs = axs.flatten()

        # Containers for arrow and text artists
        self.axes = {}
        self.quiver_current = {}
        self.quiver_desired = {}
        self.text_current = {}
        self.text_target = {}

        for idx, mod in enumerate(self.modules):
            ax = axs[idx]
            ax.set_aspect('equal')
            ax.set_xlim(-1.2, 1.2)
            ax.set_ylim(-1.2, 1.2)

            # Draw circle boundary
            circle = Circle((0, 0), radius=1.0, fill=False, color='black', linewidth=1.5)
            ax.add_patch(circle)

            # Initial arrows: zero-length (we’ll update immediately)
            u_c, v_c = 0.0, 0.0  # current arrow vector
            u_d, v_d = 0.0, 0.0  # desired arrow vector

            # Blue arrow for current angle
            q_c = ax.quiver(
                0, 0, u_c, v_c,
                angles='xy', scale_units='xy', scale=1,
                color='blue', width=0.02, label='Current'
            )
            # Green arrow for desired angle
            q_d = ax.quiver(
                0, 0, u_d, v_d,
                angles='xy', scale_units='xy', scale=1,
                color='green', width=0.02, label='Desired'
            )
            self.quiver_current[mod] = q_c
            self.quiver_desired[mod] = q_d

            # Text fields (top-left inside circle)
            txt_c = ax.text(
                -1.1, 1.05,
                "0.00",
                color='blue',
                fontsize=10,
                weight='bold'
            )
            txt_d = ax.text(
                -1.1, 0.85,
                "0.00",
                color='green',
                fontsize=10,
                weight='bold'
            )
            self.text_current[mod] = txt_c
            self.text_target[mod] = txt_d

            ax.set_title(mod.replace('_', ' ').title())
            ax.axis('off')  # Hide ticks

            self.axes[mod] = ax

        # Draw initial figure
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run_gui(self):
        """
        Enters a loop that updates the GUI at ~10 Hz.
        This call blocks until the Matplotlib window is closed.
        """
        rate = self.create_rate(10)  # 10 Hz
        try:
            while rclpy.ok():
                self.update_gui()
                plt.pause(0.001)  # Process Matplotlib events
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            plt.close(self.fig)

    def update_gui(self):
        """
        Updates arrows and text for each module based on current vs. target values.
        Called from the GUI loop (~10 Hz).
        """

        for mod in self.modules:
            # Fetch current & target angles (default to 0 if None)
            curr_ang = self.current_steering_angles.get(mod) or 0.0
            tar_ang  = self.target_steering_angles.get(mod)  or 0.0

            # Fetch current & target velocities (default to 0)
            curr_vel = self.current_drive_velocities.get(mod) or 0.0
            tar_vel  = self.target_drive_speeds.get(mod)   or 0.0

            # Compute normalized speeds (clamp at 4.0)
            norm_c = min(abs(curr_vel), 4.0) / 4.0
            norm_d = min(abs(tar_vel), 4.0) / 4.0

            # Scale each between 30%–100% of max length (0.9)
            scale_c = 0.3 + 0.7 * norm_c
            scale_d = 0.3 + 0.7 * norm_d

            # Final arrow lengths
            length_c = scale_c * 0.9
            length_d = scale_d * 0.9

            # If velocity is negative, display angle + π (so arrow points opposite)
            disp_curr_ang = curr_ang + (math.pi if curr_vel < 0 else 0.0)
            disp_tar_ang  = tar_ang  + (math.pi if tar_vel  < 0 else 0.0)

            # Mapping so that:
            #   • angle=0 → arrow points up (0, +length)
            #   • positive angle rotates CCW: angle=+π/2 → left, angle=π → down, angle=−π/2 → right.
            u_c = -math.sin(disp_curr_ang) * length_c
            v_c =  math.cos(disp_curr_ang) * length_c
            u_d = -math.sin(disp_tar_ang)  * length_d
            v_d =  math.cos(disp_tar_ang)  * length_d

            # Update quiver arrows
            self.quiver_current[mod].set_UVC(u_c, v_c)
            self.quiver_desired[mod].set_UVC(u_d, v_d)

            # Update velocity texts
            self.text_current[mod].set_text(f"{curr_vel:.2f}")
            self.text_target[mod].set_text(f"{tar_vel:.2f}")

        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def cmd_vel_callback(self, msg: Twist):
        """
        On new /cmd_vel:
          1) Compute (raw_angle_i, raw_speed_i) for each module via swerve kinematics.
          2) Optimize: if |raw_angle_i| > 90°, set angle_i = raw_angle_i ± π (to bring into ±90°)
             and flip speed_i sign.
          3) If all steering axes are already within tolerance of the optimized angles
             AND all_steer_ready == True, just publish updated drive speeds.
          4) Otherwise, stop all drives, set new optimized targets, publish steering setpoints,
             and wait for check_all_steer_and_publish_drives() to release drives.
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z  # yaw rate

        # 1) Compute per‐module raw angle_i & speed_i
        raw_angles = {}
        raw_speeds = {}
        for mod, (x_i, y_i) in self.module_positions.items():
            Vx_i = vx - omega * y_i
            Vy_i = vy + omega * x_i
            speed_i = math.hypot(Vx_i, Vy_i)
            angle_i = math.atan2(Vy_i, Vx_i)
            raw_angles[mod] = angle_i
            raw_speeds[mod] = speed_i

        # 2) Optimize angle to ±90° range (±π/2). If outside, subtract/add π and invert speed.
        new_angles = {}
        new_speeds = {}
        for mod in self.modules:
            ang = raw_angles[mod]
            spd = raw_speeds[mod]

            # If angle > +90° (π/2), subtract π and invert speed
            if ang > math.pi/2:
                ang -= math.pi
                spd = -spd
            # If angle < −90° (−π/2), add π and invert speed
            elif ang < -math.pi/2:
                ang += math.pi
                spd = -spd

            # Now ang is guaranteed in [-π/2, +π/2]
            new_angles[mod] = ang
            new_speeds[mod] = spd

        # 3) If previously all_steer_ready and current angles already within tolerance of new_angles,
        #    then immediately publish new drive speeds and update target_steering_angles for next check.
        if self.all_steer_ready:
            all_within = True
            for mod in self.modules:
                curr_ang = self.current_steering_angles.get(mod)
                if curr_ang is None:
                    all_within = False
                    break
                err = abs(self._angle_diff(new_angles[mod], curr_ang))
                if err > self.steering_tolerance:
                    all_within = False
                    break

            if all_within:
                # Publish new drive speeds immediately
                for mod in self.modules:
                    self.target_drive_speeds[mod] = new_speeds[mod]
                    drive_msg = Float64MultiArray()
                    drive_msg.data = [self.target_drive_speeds[mod]]
                    self.drive_pubs[mod].publish(drive_msg)
                self.get_logger().info('All steers within tolerance → published new drive speeds.')

                # Update steering targets too so future cmd_vel comparisons use them
                for mod in self.modules:
                    self.target_steering_angles[mod] = new_angles[mod]
                return

        # 4) Otherwise, re‐steer: stop all drives, update targets, publish steering
        for mod in self.modules:
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0]
            self.drive_pubs[mod].publish(stop_msg)
        self.get_logger().info('Published 0.0 to all 4 drives to stop before re‐steering.')

        # Update target angles & speeds, reset ready flags
        for mod in self.modules:
            self.target_steering_angles[mod] = new_angles[mod]
            self.target_drive_speeds[mod] = new_speeds[mod]
            self.steer_ready[mod] = False
        self.all_steer_ready = False

        # Publish new steering setpoints
        for mod in self.modules:
            steer_msg = Float64MultiArray()
            steer_msg.data = [self.target_steering_angles[mod]]
            self.steer_pubs[mod].publish(steer_msg)
        self.get_logger().info('Published new steering targets for all 4 modules (optimized to ±90°).')

    def joint_states_callback(self, msg: JointState):
        """
        Update current steering angles & drive velocities from /joint_states.
        Each JointState name[] should include the 8 relevant joints (4 steer + 4 drive).
        """
        for i, name in enumerate(msg.name):
            # Steering joints
            for mod in self.modules:
                if name == self.steer_joint_names[mod]:
                    self.current_steering_angles[mod] = msg.position[i]
            # Drive joints (velocity)
            for mod in self.modules:
                if name == self.drive_joint_names[mod]:
                    self.current_drive_velocities[mod] = msg.velocity[i]

    def check_all_steer_and_publish_drives(self):
        """
        Timer callback (10 Hz) that checks if all steering angles are within tolerance.
        If so, and if we haven't yet published drive speeds for this target, do so now.
        """
        # If no target set for any module yet, skip
        if any(self.target_steering_angles[mod] is None for mod in self.modules):
            return

        # If we already marked all_steer_ready, skip
        if self.all_steer_ready:
            return

        # If we lack current info for any module, skip
        if any(self.current_steering_angles[mod] is None for mod in self.modules):
            return

        # Check each module
        all_now_ready = True
        for mod in self.modules:
            err = abs(self._angle_diff(
                self.target_steering_angles[mod],
                self.current_steering_angles[mod]
            ))
            if err <= self.steering_tolerance:
                self.steer_ready[mod] = True
            else:
                self.steer_ready[mod] = False
                all_now_ready = False

        if all_now_ready:
            self.all_steer_ready = True
            # Publish drive speeds for all modules
            for mod in self.modules:
                drive_msg = Float64MultiArray()
                drive_msg.data = [self.target_drive_speeds[mod]]
                self.drive_pubs[mod].publish(drive_msg)
            self.get_logger().info('All steers within ±tolerance → published all 4 drive speeds.')

    @staticmethod
    def _angle_diff(goal: float, current: float) -> float:
        """
        Shortest signed difference between two angles (radians), wrapped to [−π, +π].
        """
        a = (goal - current) % (2.0 * math.pi)
        if a > math.pi:
            a -= 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveController()

    # ROS spin is handled in a background thread; GUI loop is blocking in main.
    try:
        rclpy.spin(node)  # In case any clean‐up is needed (though run_gui blocks).
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
