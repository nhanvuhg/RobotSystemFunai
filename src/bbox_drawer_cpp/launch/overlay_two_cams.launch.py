#!/usr/bin/env python3
"""
overlay_two_cams.launch.py
Launch file for dual camera bbox overlay visualization
Matches cam0/cam1 convention
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'shared_image_topic',
            default_value='/ai/image_overlay',
            description='Shared input image topic from CSI camera'
        ),
        DeclareLaunchArgument(
            'cam0_boxes_topic',
            default_value='/cam0Funai/yolo/bounding_boxes',
            description='Camera 0 (Input Tray) YOLO detections'
        ),
        DeclareLaunchArgument(
            'cam1_boxes_topic',
            default_value='/cam1Funai/yolo/bounding_boxes',
            description='Camera 1 (Output Tray) YOLO detections'
        ),
        DeclareLaunchArgument(
            'output_width',
            default_value='640',
            description='Output image width'
        ),
        DeclareLaunchArgument(
            'output_height',
            default_value='480',
            description='Output image height'
        ),
    ]

    # Get launch configurations
    shared_image = LaunchConfiguration('shared_image_topic')
    cam0_boxes = LaunchConfiguration('cam0_boxes_topic')
    cam1_boxes = LaunchConfiguration('cam1_boxes_topic')
    out_width = LaunchConfiguration('output_width')
    out_height = LaunchConfiguration('output_height')

    # Bbox drawer node
    bbox_drawer_node = Node(
        package='bbox_drawer_cpp',
        executable='overlay_bboxes_node',
        name='overlay_dual_cam',
        output='screen',
        parameters=[{
            # Camera 0 (Input Tray) parameters
            'cam0.image_topic': shared_image,
            'cam0.boxes_topic': cam0_boxes,
            'cam0.output_topic': '/cam0Funai/image_overlay',
            'cam0.output_width': out_width,
            'cam0.output_height': out_height,
            
            # Camera 1 (Output Tray) parameters
            'cam1.image_topic': shared_image,
            'cam1.boxes_topic': cam1_boxes,
            'cam1.output_topic': '/cam1Funai/image_overlay',
            'cam1.output_width': out_width,
            'cam1.output_height': out_height,
        }],
    )

    return LaunchDescription(launch_args + [bbox_drawer_node])