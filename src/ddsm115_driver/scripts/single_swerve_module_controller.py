#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class SingleSwerveModuleController(Node):
    def __init__(self):
        super().__init__('single_swerve_module_controller')

        # === PARAMETERS ===
        self.declare_parameter('steering_joint_name', 'front_left_steer')
        self.declare_parameter('traction_joint_name', 'front_left_drive')

        self.declare_parameter('steering_cmd_topic', '/front_left_steer_controller/commands')
        self.declare_parameter('traction_cmd_topic', '/front_left_drive_controller/commands')

        self.declare_parameter('steering_tolerance_deg', 2.0)

        # === PULLED PARAMETERS ===
        self.steering_joint_name = self.get_parameter('steering_joint_name').get_parameter_value().string_value
        self.traction_joint_name = self.get_parameter('traction_joint_name').get_parameter_value().string_value

        self.steering_cmd_topic = self.get_parameter('steering_cmd_topic').get_parameter_value().string_value
        self.traction_cmd_topic = self.get_parameter('traction_cmd_topic').get_parameter_value().string_value

        self.steering_tolerance = math.radians(
            self.get_parameter('steering_tolerance_deg').get_parameter_value().double_value
        )

        # === INTERNAL STATE ===
        self.current_steering_angle = None
        self.current_traction_velocity = None

        self.target_steering_angle = None
        self.target_traction_velocity = None

        self.steering_ready = False

        # === SUBSCRIBERS ===
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )

        # === PUBLISHERS ===
        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            self.steering_cmd_topic,
            10
        )
        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            self.traction_cmd_topic,
            10
        )

        # Timer to check steering readiness and then publish traction
        self.check_timer = self.create_timer(0.1, self.check_steering_and_publish_traction)

        self.get_logger().info('SingleSwerveModuleController (with immediate traction‐stop) started.')

    def cmd_vel_callback(self, msg: Twist):
        """
        Upon receiving a new cmd_vel:
          1. Immediately publish zero to traction (stop drive motor).
          2. Store new steering + traction targets.
          3. Publish new steering command.
          4. Defer publishing traction until steering is within tolerance.
        """
        desired_speed = msg.linear.x
        desired_angle = msg.angular.z  # [rad]  ← Adjust this mapping if you want to test other angles

        self.get_logger().info(
            f'← New cmd_vel: linear.x={desired_speed:.2f}, angular.z={msg.angular.z:.2f}'
        )

        # 2) Store the new targets
        self.target_steering_angle = desired_angle
        self.target_traction_velocity = desired_speed

        # 1) Immediately command traction = 0.0 to stop any ongoing driving
        error = self._angle_diff(self.target_steering_angle, self.current_steering_angle)
        if abs(error) >= self.steering_tolerance:
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.0]
            self.traction_pub.publish(stop_msg)
            self.get_logger().info('→ Immediately published traction=0.0 to stop drive motor.')

        # Reset the “steering_ready” flag so we wait for the new angle
        self.steering_ready = False

        # 3) Publish the new steering target (position)
        steering_msg = Float64MultiArray()
        steering_msg.data = [self.target_steering_angle]
        self.steering_pub.publish(steering_msg)
        self.get_logger().info(f'→ Published steering target: {math.degrees(desired_angle):.1f}°')

        # 4) Do NOT yet publish traction velocity;
        #    check_steering_and_publish_traction() will send the new speed once ready.

    def joint_states_callback(self, msg: JointState):
        """
        Update current steering position & traction velocity from /joint_states.
        """
        for i, name in enumerate(msg.name):
            if name == self.steering_joint_name:
                self.current_steering_angle = msg.position[i]
            if name == self.traction_joint_name:
                self.current_traction_velocity = msg.velocity[i]

    def check_steering_and_publish_traction(self):
        """
        Every 0.1s, check if steering has reached the new target (± tolerance).
        If yes, and if we haven’t yet sent the new traction command, send it now.
        """
        # If no target has been set yet, skip
        if self.target_steering_angle is None:
            return

        # If we already published the new traction for this target, skip
        if self.steering_ready:
            return

        # If we haven’t seen any joint_states yet, skip
        if self.current_steering_angle is None:
            return

        # Compute wrapped angle error
        error = self._angle_diff(self.target_steering_angle, self.current_steering_angle)
        if abs(error) <= self.steering_tolerance:
            self.steering_ready = True
            self.get_logger().info('Steering within ±2° → publishing new traction command.')

            traction_msg = Float64MultiArray()
            traction_msg.data = [self.target_traction_velocity]
            self.traction_pub.publish(traction_msg)
            self.get_logger().info(f'→ Published traction target: {self.target_traction_velocity:.2f}')
        else:
            # Still turning toward target; do nothing
            pass

    @staticmethod
    def _angle_diff(goal: float, current: float) -> float:
        """
        Compute shortest signed difference between two angles (radians),
        wrapped to [−π, +π].
        """
        a = (goal - current) % (2.0 * math.pi)
        if a > math.pi:
            a -= 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = SingleSwerveModuleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down SingleSwerveModuleController...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
