# Multi-camera YOLO + HailoRT (2 cams)
# - cam0: /dev/video0  → topics /cam0/...
# - cam1: /dev/video1  → topics /cam1/...

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Common args (dùng chung cho 2 camera)
    launch_args = [
        DeclareLaunchArgument('model_path',
            default_value='./src/hailort_yolo_common/weights/yolox_tiny.hef',
            description='HEF model path'),
        DeclareLaunchArgument('class_labels_path',
            default_value='',
            description='path to labels.txt (optional)'),
        DeclareLaunchArgument('conf',
            default_value='0.20',
            description='YOLO confidence threshold'),
        DeclareLaunchArgument('nms',
            default_value='0.35',
            description='YOLO NMS IoU threshold'),
        DeclareLaunchArgument('imshow_isshow',
            default_value='false',
            description='do not pop up GUI windows when multi-cam'),
        DeclareLaunchArgument('publish_resized_image',
            default_value='false',
            description='publish resized image instead of original'),
        DeclareLaunchArgument('nms_output_name',
            default_value='yolov8s/yolov8_nms_postprocess',
            description='Exact NMS output tensor name inside the HEF'),
    ]

    # Riêng cho từng camera
    cam0_args = {
        'video_device': '/dev/video0',
        'namespace':    'cam0',
        'camera_frame': 'cam0_frame'
    }
    cam1_args = {
        'video_device': '/dev/video4',
        'namespace':    'cam1',
        'camera_frame': 'cam1_frame'
    }

    def make_cam_nodes(ns: str, dev: str, frame_id: str):
        # usb_cam
        usb = ComposableNode(
            package='usb_cam',
            plugin='usb_cam::UsbCamNode',
            name=f'usb_cam_{ns}',
            namespace=f'/{ns}',
            parameters=[{
                'video_device': dev,
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',   # đổi 'yuyv' nếu cam không hỗ trợ mjpeg
                'camera_frame_id': frame_id,
                'brightness': 100
            }],
            remappings=[('image', 'image_raw')]
        )

        # yolo
        yolo = ComposableNode(
            package='yolo_ros_hailort_cpp',
            plugin='yolo_ros_hailort_cpp::YoloNode',
            name=f'yolo_{ns}',
            namespace=f'/{ns}',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'class_labels_path': LaunchConfiguration('class_labels_path'),
                'conf': LaunchConfiguration('conf'),
                'nms': LaunchConfiguration('nms'),
                'imshow_isshow': LaunchConfiguration('imshow_isshow'),
                'publish_resized_image': LaunchConfiguration('publish_resized_image'),
                'nms_output_name': LaunchConfiguration('nms_output_name'),
                # topic names cho mỗi namespace
                'src_image_topic_name': f'/{ns}/image_raw',
                'publish_image_topic_name': f'/{ns}/yolo/image_raw',
                'publish_boundingbox_topic_name': f'/{ns}/yolo/bounding_boxes',
            }]
        )
        return [usb, yolo]

    container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=
            make_cam_nodes(cam0_args['namespace'], cam0_args['video_device'], cam0_args['camera_frame'])
            + make_cam_nodes(cam1_args['namespace'], cam1_args['video_device'], cam1_args['camera_frame']),
        output='screen'
    )

    return launch.LaunchDescription(launch_args + [container])
