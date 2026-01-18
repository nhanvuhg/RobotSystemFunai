from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    param_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'joint_pose_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='robot_control_main',
            executable='robot_logic_node',
            name='robot_logic_node',
            parameters=[param_file],
            output='screen'
        )
    ])
