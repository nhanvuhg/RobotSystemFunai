from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cam0 = Node(
        package='rpi_cam_bridge',
        executable='cam_node',
        name='cam0_node',
        parameters=[
            {'camera_id': 0},
            {'width': 640},
            {'height': 480},
            {'fps': 30},
            {'topic': '/cam0/image_raw'},
            {'frame_id': 'cam0_frame'}
        ],
        output='screen'
    )

    # cam1 = Node(
    #     package='rpi_cam_bridge',
    #     executable='cam_node',
    #     name='cam1_node',
    #     parameters=[
    #         {'camera_id': 1},
    #         {'width': 640},
    #         {'height': 480},
    #         {'fps': 30},
    #         {'topic': '/cam1/image_raw'},
    #         {'frame_id': 'cam1_frame'}
    #     ],
    #     output='screen'
    # )

    #return LaunchDescription([cam0, cam1])
    return LaunchDescription([cam0])
