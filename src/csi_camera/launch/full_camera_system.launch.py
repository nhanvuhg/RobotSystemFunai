#!/usr/bin/env python3
"""
full_camera_system.launch.py
Complete camera + YOLO + bbox drawer system
Start once, robot controls camera switching via /robot/camera_select
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # ================================================================
    # 1. CSI CAMERA NODE
    # ================================================================
    # Publishes to /cam0/image_raw or /cam1/image_raw based on active camera
    csi_camera_node = Node(
        package='csi_camera',
        executable='csi_camera_node',
        name='csi_camera_node',
        output='screen',
        parameters=[{
            'width': 1280,
            'height': 720,
            'fps': 30,
            # 'output_topic' is not used; node manages /cam0/image_raw and /cam1/image_raw internally
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    # ================================================================
    # 2. YOLO CONTAINER WITH 2 MODELS (cam0 + cam1)
    # ================================================================
    yolo_container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # YOLO cam0 (Input Tray Model) - Listens to /cam0/image_raw
            ComposableNode(
                package='yolo_ros_hailort_cpp',
                plugin='yolo_ros_hailort_cpp::YoloNode',
                name='yolo_cam0',
                parameters=[{
                    'model_path': '/home/pi/input_1_yolov8s.hef',
                    'src_image_topic_name': '/cam0/image_raw', # Only runs when Cam 0 is active
                    'publish_boundingbox_topic_name': '/cam0/yolo/bounding_boxes',
                    'publish_image_topic_name': '/cam0/yolo/image_raw',
                    'conf': 0.25, # Lower confidence for better pickup
                    'publish_resized_image': False,
                }]
            ),
            # YOLO cam1 (Output Tray Model) - Listens to /cam1/image_raw
            ComposableNode(
                package='yolo_ros_hailort_cpp',
                plugin='yolo_ros_hailort_cpp::YoloNode',
                name='yolo_cam1',
                parameters=[{
                    'model_path': '/home/pi/yolov8s.hef',
                    'src_image_topic_name': '/cam1/image_raw', # Only runs when Cam 1 is active
                    'publish_boundingbox_topic_name': '/cam1/yolo/bounding_boxes',
                    'publish_image_topic_name': '/cam1/yolo/image_raw',
                    'conf': 0.25, 
                    'publish_resized_image': False,
                }]
            ),
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    
    # ================================================================
    # 3. BBOX DRAWER (Visualization)
    # ================================================================
    # Draws boxes on images for visualization/debugging
    bbox_drawer_node = Node(
        package='bbox_drawer_cpp',
        executable='overlay_bboxes_node',
        name='overlay_dual_cam',
        output='screen',
        parameters=[{
            # Camera 0 (Input Tray)
            'cam0.image_topic': '/cam0/image_raw',
            'cam0.boxes_topic': '/cam0/yolo/bounding_boxes',
            'cam0.output_topic': '/cam0/image_overlay',
            'cam0.output_width': 640,
            'cam0.output_height': 480,
            
            # Camera 1 (Output Tray)
            'cam1.image_topic': '/cam1/image_raw',
            'cam1.boxes_topic': '/cam1/yolo/bounding_boxes',
            'cam1.output_topic': '/cam1/image_overlay',
            'cam1.output_width': 640,
            'cam1.output_height': 480,
        }],
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        csi_camera_node,
        yolo_container,
        bbox_drawer_node,
    ])
