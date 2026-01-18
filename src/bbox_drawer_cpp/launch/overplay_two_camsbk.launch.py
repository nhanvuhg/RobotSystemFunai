from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bbox_drawer_cpp',
            executable='overlay_bboxes_node',
            name='overlay_two_cams',
            output='screen',
            parameters=[{
                'cam0.image_topic': '/cam0/image_raw',
                'cam0.boxes_topic': '/cam0/yolo/bounding_boxes',
                'cam0.output_topic': '/cam0/image_overlay',
                'cam0.output_width':  1280, #640
                'cam0.output_height': 720, #480
                'cam1.image_topic': '/cam1/image_raw',
                'cam1.boxes_topic': '/cam1/yolo/bounding_boxes',
                'cam1.output_topic': '/cam1/image_overlay',
                'cam1.output_width':  1280,
                'cam1.output_height': 720,
                'cam2.image_topic': '/cam2/image_raw',
                'cam2.boxes_topic': '/cam2/yolo/bounding_boxes',
                'cam2.output_topic': '/cam2/image_overlay',
                'cam2.output_width':  640,
                'cam2.output_height': 480,
            }]
        )
    ])