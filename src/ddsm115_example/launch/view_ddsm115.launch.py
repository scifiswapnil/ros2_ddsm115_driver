#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Locate files
    pkg_share = get_package_share_directory('ddsm115_example')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ddsm115_example.urdf.xacro')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'tf.rviz')
    ct_cfg     = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # 2) Build the robot_description parameter
    robot_desc = {
      'robot_description': ParameterValue(
         Command(['xacro ', urdf_file]), value_type=str
      )
    }

    # 3) Publish TF and URDF on /robot_description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
          robot_desc,
          {'publish_robot_description': True},
        ],
    )

    # 4) Bring up ros2_control_node with URDF + controller YAML
    ctrl = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[ robot_desc, ct_cfg ],
        remappings=[
          ('~/robot_description', '/robot_description'),
        ],
    )

    # 5) RViz
    viz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    # 6) Spawn each controller separately, with distinct variable names
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_front_left_steer = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_front_left_steer_controller',
        arguments=['front_left_steer_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_front_left_drive = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_front_left_drive_controller',
        arguments=['front_left_drive_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_front_right_steer = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_front_right_steer_controller',
        arguments=['front_right_steer_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_front_right_drive = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_front_right_drive_controller',
        arguments=['front_right_drive_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_rear_left_steer = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_rear_left_steer_controller',
        arguments=['rear_left_steer_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_rear_left_drive = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_rear_left_drive_controller',
        arguments=['rear_left_drive_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_rear_right_steer = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_rear_right_steer_controller',
        arguments=['rear_right_steer_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_rear_right_drive = Node(
        package='controller_manager',
        executable='spawner',
        name='spawn_rear_right_drive_controller',
        arguments=['rear_right_drive_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.5',
        description='Wheelbase (distance front-to-back) in meters'
    )

    track_width_arg = DeclareLaunchArgument(
        'track_width',
        default_value='0.5',
        description='Track width (distance left-to-right) in meters'
    )
    steering_tol_arg = DeclareLaunchArgument(
        'steering_tolerance_deg',
        default_value='2.0',
        description='Steering tolerance in degrees (± value)'
    )
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='true',
        description='Whether to launch the Matplotlib GUI (true/false)'
    )

    # Node configuration
    swerve_node = Node(
        package='ddsm115_driver',               # ← Replace with your actual package name
        executable='swerve_drive_controller.py', # ← Replace with the actual executable name (without “.py”)
        name='swerve_drive_controller',
        output='screen',
        parameters=[{
            'wheelbase': LaunchConfiguration('wheelbase'),
            'track_width': LaunchConfiguration('track_width'),
            'steering_tolerance_deg': LaunchConfiguration('steering_tolerance_deg'),
            'enable_gui': LaunchConfiguration('enable_gui')
        }]
    )


    return LaunchDescription([
        rsp,
        ctrl,
        spawn_jsb,
        spawn_front_left_steer,
        spawn_front_left_drive,
        spawn_front_right_steer,
        spawn_front_right_drive,
        spawn_rear_left_steer,
        spawn_rear_left_drive,
        spawn_rear_right_steer,
        spawn_rear_right_drive,
        viz,
        wheelbase_arg,
        track_width_arg,
        steering_tol_arg,
        enable_gui_arg,
        swerve_node
    ])
