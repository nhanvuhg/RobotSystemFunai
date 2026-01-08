#!/usr/bin/env python3
"""
Launch file for robot control with Festo gripper integration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Parameters for gripper node
    cpx_ip_arg = DeclareLaunchArgument(
        'cpx_ip',
        default_value='192.168.27.93',
        description='IP address of Festo CPX-AP module'
    )
    
    cpx_module_arg = DeclareLaunchArgument(
        'cpx_module',
        default_value='1',
        description='CPX module index (usually 1 for CPX-AP-8DI)'
    )
    
    # Robot logic node (C++)
    robot_logic_node = Node(
        package='robot_control_main',
        executable='robot_logic_node',
        name='robot_logic_node',
        parameters=[
            'src/robot_control_main/config/joint_pose_params.yaml'
        ],
        output='screen'
    )
    
    # Festo gripper controller node (Python)
    festo_gripper_node = Node(
        package='robot_control_main',
        executable='gripper_festo_node.py',
        name='festo_gripper_controller',
        parameters=[
            {'cpx_ip': LaunchConfiguration('cpx_ip')},
            {'cpx_module_index': LaunchConfiguration('cpx_module')}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        cpx_ip_arg,
        cpx_module_arg,
        robot_logic_node,
        festo_gripper_node
    ])
