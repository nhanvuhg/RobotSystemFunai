from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nova5_gui',
            executable='nova5_gui_node',
            name='nova5_gui',
            output='screen'
        )
    ])
