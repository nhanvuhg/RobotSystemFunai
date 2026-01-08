#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RPiCamNode(Node):
    def __init__(self):
        super().__init__('rpi_cam_node')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'rpi_cam')
        self.cam_id = int(self.get_parameter('camera_id').value)
        self.W = int(self.get_parameter('width').value)
        self.H = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.get_logger().info(f"Starting rpicam-vid for camera {self.cam_id} {self.W}x{self.H}@{self.fps} -> {self.topic}")
        self.pub = self.create_publisher(Image, self.topic, 10)
        self.bridge = CvBridge()
        cmd = [
            "rpicam-vid",
            "--camera", str(self.cam_id),
            "--width", str(self.W),
            "--height", str(self.H),
            "--framerate", str(self.fps),
            "--codec", "yuv420",
            "--timeout", "0",
            "--stdout"
        ]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        self.y_size = self.W * self.H
        self.uv_size = (self.W // 2) * (self.H // 2)
        self.frame_size = self.y_size + 2 * self.uv_size
        period = 1.0 / max(self.fps, 1)
        self.timer = self.create_timer(period, self.timer_cb)

    def timer_cb(self):
        raw = self.proc.stdout.read(self.frame_size)
        if len(raw) != self.frame_size:
            self.get_logger().warn("Short read / camera stopped?")
            return
        y_plane = np.frombuffer(raw[0:self.y_size], dtype=np.uint8).reshape((self.H, self.W))
        u_plane = np.frombuffer(raw[self.y_size:self.y_size + self.uv_size], dtype=np.uint8).reshape((self.H // 2, self.W // 2))
        v_plane = np.frombuffer(raw[self.y_size + self.uv_size:self.frame_size], dtype=np.uint8).reshape((self.H // 2, self.W // 2))
        u_up = cv2.resize(u_plane, (self.W, self.H), interpolation=cv2.INTER_NEAREST)
        v_up = cv2.resize(v_plane, (self.W, self.H), interpolation=cv2.INTER_NEAREST)
        yuv = cv2.merge([y_plane, v_up, u_up])
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YCrCb2BGR)
        msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'proc'):
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1.0)
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPiCamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
