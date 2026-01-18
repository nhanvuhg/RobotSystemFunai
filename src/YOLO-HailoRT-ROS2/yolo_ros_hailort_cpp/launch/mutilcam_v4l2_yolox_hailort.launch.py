from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # === Khai báo 2 model_path riêng biệt ===
    launch_args = [
        DeclareLaunchArgument(
            'model_path_cam0',
            default_value='/home/pi/yolov8s.hef',
            description='HEF model path cho cam0'
        ),
        DeclareLaunchArgument(
            'model_path_cam1',
            default_value='/home/pi/input_1_yolov8s.hef',
            description='HEF model path cho cam1'
        ),
        DeclareLaunchArgument(
            'model_path_cam2',
            default_value='/home/pi/input_1_yolov8s.hef',
            description='HEF model path cho cam2'
        ),
        DeclareLaunchArgument('class_labels_path', default_value=''),
        DeclareLaunchArgument('conf', default_value='0.4'),
        DeclareLaunchArgument('nms', default_value='0.45'),
        DeclareLaunchArgument('imshow_isshow', default_value='false'),
        DeclareLaunchArgument('publish_resized_image', default_value='false'),
        DeclareLaunchArgument(
            'nms_output_name',
            default_value='yolov8s/yolov8_nms_postprocess'
        ),
    ]

    # ================== 2 node v4l2_camera ==================
    cam0_v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam0_usb',
        namespace='cam0',
        parameters=[{
            "video_device": "/dev/video0",
            "image_size": [1280, 720],
            "framerate": 5.0,
            "output_encoding": "rgb8",
            "camera_info_url": "file:////home/pi/.ros/camera_info/default_cam.yaml",
        }]
    )

    cam1_v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam1_usb',
        namespace='cam1',
        parameters=[{
            "video_device": "/dev/video8",
            "image_size": [1280, 720],
            "framerate": 5.0,
            "output_encoding": "rgb8",
            "camera_info_url": "file:////home/pi/.ros/camera_info/default_cam.yaml",
        }]
    )

    cam2_v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='cam2_usb',
        namespace='cam2',
        parameters=[{
            "video_device": "/dev/video4",
            "image_size": [1280, 720],
            "framerate": 5.0,
            "output_encoding": "rgb8",
            "camera_info_url": "file:////home/pi/.ros/camera_info/default_cam.yaml",
        }]
    )

    # ================== YOLO Node cho từng camera ==================
    def make_yolo_node(ns: str, model_path: LaunchConfiguration):
        return ComposableNode(
            package='yolo_ros_hailort_cpp',
            plugin='yolo_ros_hailort_cpp::YoloNode',
            name=f'yolo_{ns}',
            namespace=f'/{ns}',
            parameters=[{
                'model_path': model_path,   # ⭐ Model riêng cho từng cam
                'class_labels_path': LaunchConfiguration('class_labels_path'),
                'conf': LaunchConfiguration('conf'),
                'nms': LaunchConfiguration('nms'),
                'imshow_isshow': LaunchConfiguration('imshow_isshow'),
                'publish_resized_image': LaunchConfiguration('publish_resized_image'),
                'nms_output_name': LaunchConfiguration('nms_output_name'),
                # Topic
                'src_image_topic_name': f'/{ns}/image_raw',
                'publish_image_topic_name': f'/{ns}/yolo/image_raw',
                'publish_boundingbox_topic_name': f'/{ns}/yolo/bounding_boxes',
            }]
        )

    yolo_container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            make_yolo_node('cam0', LaunchConfiguration('model_path_cam0')),
            make_yolo_node('cam1', LaunchConfiguration('model_path_cam1')),
            #make_yolo_node('cam2', LaunchConfiguration('model_path_cam2')),
        ],
        output='screen'
    )

    return LaunchDescription(
        launch_args + [
            cam0_v4l2,
            cam1_v4l2,
            #cam2_v4l2,
            yolo_container
        ]
    )
