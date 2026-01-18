from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='snap7_driver',
            executable='snap7_node',
            name='snap7_node',
            output='screen',
            parameters=[],
            remappings=[
            ]
        )
    ])


# PARAMETERS:
# db_number: So cua DB can doc
# offset: Vi tri bat dau doc (vi du doc vi tri 5 => offset = 8, doc vi tri 1 => offset = 0)
# size: So luong vi tri can doc (khong can x2) (vi du doc 5 vi tri => size =5)

# Lenh doc thanh DB PLC: 
#   ros2 topic pub --once /plc/read_request plc_msg/msg/PLCRead "{plc_ip: '192.168.27.9', db_number: 1, start: 0, size: 5}"
# Lenh ghi vao DB PLC
#   ros2 topic pub --once /plc/write_request plc_msg/msg/PLCWrite "{plc_ip: '192.168.27.6', db_number: 1, start: 0, values: [123, 456]}"

# subscribe topic nay de nhan ket qua doc tu PLC (kich hoat khi co lenh doc tu topic /plc/read_request):
#   ros2 topic echo /plc/read_response