from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_control_main')
    params_file = os.path.join(pkg_dir, 'config', 'joint_pose_params.yaml')
    
    return LaunchDescription([
        Node(
            package='robot_control_main',
            executable='robot_logic_node',
            name='robot_logic_nova5',
            output='screen',
            parameters=[
                params_file,
                {'robot_ip': '192.168.27.8'}  # ← Thêm IP đúng
            ]
        )
    ])