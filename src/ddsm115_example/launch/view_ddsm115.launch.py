#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Locate files
    pkg_share = get_package_share_directory('ddsm115_example')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ddsm115_example.urdf.xacro')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'tf.rviz')
    # hw_cfg  = os.path.join(pkg_share, 'config','hardware.yaml')
    ct_cfg  = os.path.join(pkg_share, 'config','controllers.yaml')    

    # 2) Build the robot_description parameter
    robot_desc = {
      'robot_description': ParameterValue(
         Command(['xacro ', urdf_file]), value_type=str
      )
    }

    # 3) Publish TF *and* the URDF on /robot_description
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

    # 4) Bring up ros2_control_node with *only* the URDF param
    ctrl = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[ robot_desc, ct_cfg],
        remappings=[
          # Remap its private topic to the global one
          ('~/robot_description', '/robot_description'),
        ],
    )
    # 5) Launch RViz so you can see the model if you like
    viz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )



    spawn_pos = Node(
        package='controller_manager',
        executable='spawner',
        name='front_left_steer_controller',
        arguments=['front_left_steer_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )
    spawn_vel = Node(
        package='controller_manager',
        executable='spawner',
        name='front_left_drive_controller',
        arguments=['front_left_drive_controller', 
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([rsp, ctrl, spawn_jsb, spawn_pos, spawn_vel, viz])
