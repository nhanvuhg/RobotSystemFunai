from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for DUAL PARALLEL camera mode
    
    This launches:
    1. csi_dual_camera_node - Runs both CAM0 and CAM1 simultaneously
    2. Two YOLO nodes (one for each camera stream)
    
    For BACKUP switch-based mode, use: full_camera_system.launch.py
    """
    return LaunchDescription([
        # Dual Camera Node (NEW - Parallel Mode)
        Node(
            package='csi_camera',
            executable='csi_dual_camera_node',
            name='csi_dual_camera_node',
            parameters=[{
                'width': 1280,
                'height': 720,
                'fps': 30,
                'cam0_topic': 'cam0Funai/image_raw',
                'cam1_topic': 'cam1Funai/image_raw',
            }],
            output='screen'
        ),
        
        # YOLO for CAM0 (Input Tray)
        Node(
            package='yolo_ros_hailort_cpp',
            executable='yolo_node',
            name='yolo_cam0_node',
            parameters=[{
                'model_path': '/home/pi/yolov8s_InputCart.hef',
            }],
            remappings=[
                ('/image_raw', '/cam0Funai/image_raw'),
                ('/yolo/detections', '/cam0/detections'),
            ],
            output='screen'
        ),
        
        # YOLO for CAM1 (Output Tray)
        Node(
            package='yolo_ros_hailort_cpp',
            executable='yolo_node',
            name='yolo_cam1_node',
            parameters=[{
                'model_path': '/home/pi/yolov8s_trayoutput.hef',
            }],
            remappings=[
                ('/image_raw', '/cam1Funai/image_raw'),
                ('/yolo/detections', '/cam1/detections'),
            ],
            output='screen'
        ),
    ])
