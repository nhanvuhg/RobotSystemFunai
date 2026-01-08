import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import subprocess
import os
import signal

def run_command(cmd):
    print(f"Executing: {cmd}")
    subprocess.run(cmd, shell=True)

def restart_system():
    print("🛑 Stopping system...")
    run_command("./stop_all.sh")
    run_command("pkill -9 -f component_container")
    time.sleep(5)
    print("🚀 Starting system...")
    subprocess.Popen("nohup ./run_all_three.sh > /tmp/auto_debug_py.log 2>&1 &", shell=True)
    time.sleep(20) # Wait for startup

def main():
    rclpy.init()
    node = Node('automated_tester')

    # QoS Profiles
    qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

    # Publishers
    pub_mode = node.create_publisher(Int32, '/robot/set_mode', qos_reliable)
    pub_start = node.create_publisher(Bool, '/system/start_button', qos_best_effort)
    pub_row = node.create_publisher(Int32, '/robot/command_row', qos_reliable)
    pub_state = node.create_publisher(String, '/robot/goto_state', qos_reliable)

    try:
        restart_system()

        print("Set Manual Mode (3)...")
        msg_mode = Int32()
        msg_mode.data = 3
        pub_mode.publish(msg_mode)
        time.sleep(2)

        print("Trigger Start Button...")
        msg_start = Bool()
        msg_start.data = True
        pub_start.publish(msg_start)
        
        print("⏳ Waiting 10s for HOME motion...")
        time.sleep(10)

        print("Send Command Row (2)...")
        msg_row = Int32()
        msg_row.data = 2
        pub_row.publish(msg_row)
        time.sleep(2)

        print("Send Goto State (INIT_LOAD_CHAMBER_DIRECT)...")
        msg_state = String()
        msg_state.data = "INIT_LOAD_CHAMBER_DIRECT"
        pub_state.publish(msg_state)

        print("✅ Commands sent. Checking logs...")
        time.sleep(5)
        run_command("tail -n 20 /home/pi/ros2_ws/logs/robot_logic_node.log")
        run_command("tail -n 20 /home/pi/ros2_ws/logs/dobot_bringup.log")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
