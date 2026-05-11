import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import numpy as np
import math


class SimpleFollower(Node):
    def __init__(self):
        super().__init__("simple_follower")

        # --- PARAMETERS ---
        self.target_distance = 1.5
        self.max_linear = 0.2
        self.max_angular = 0.2

        # Camera intrinsics
        self.fx = None
        self.cx = None

        # State
        self.center = None
        self.depth_image = None

        self.bridge = CvBridge()

        # --- SUBSCRIBERS ---
        self.create_subscription(Point, "/person_center", self.center_callback, 10)
        self.create_subscription(Image, "/camera/camera/depth/image_raw", self.depth_callback, 10)
        self.create_subscription(CameraInfo, "/camera/camera/camera_info", self.info_callback, 10)

        # --- PUBLISHER ---
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # --- TIMER ---
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Simple follower started")

    # --- CALLBACKS ---
    def center_callback(self, msg):
        self.center = (int(msg.x), int(msg.y))

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Print encoding once
        if not hasattr(self, "printed_encoding"):
            self.get_logger().info(f"Depth encoding: {msg.encoding}")
            self.printed_encoding = True

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.get_logger().info("Camera intrinsics received")

    # --- MAIN CONTROL LOOP ---
    def control_loop(self):
        if self.center is None or self.depth_image is None or self.fx is None:
            return

        u, v = self.center

        h, w = self.depth_image.shape

        # Avoid out-of-bounds
        if v < 2 or v > h - 3 or u < 2 or u > w - 3:
            return

        # --- ROBUST DEPTH (5x5 region) ---
        region = self.depth_image[v-2:v+3, u-2:u+3]

        # Remove invalid values
        region = region[np.isfinite(region)]

        if len(region) == 0:
            return

        depth = np.median(region)

        # --- HANDLE ENCODING ---
        if self.depth_image.dtype == np.uint16:
            depth = float(depth) / 1000.0  # mm → meters
        else:
            depth = float(depth)  # already meters

        # Ignore bad depth
        if depth <= 0.1 or depth > 10.0:
            return

        # --- ANGLE ---
        x = (u - self.cx) / self.fx
        angle = math.atan(x)  # radians

        # --- CONTROL ---
        dist_error = depth - self.target_distance

        # Simple proportional control
        linear = dist_error * 0.5
        angular = angle * 1.0

        # --- LIMITS ---
        linear = np.clip(linear, -self.max_linear, self.max_linear)
        angular = np.clip(angular, -self.max_angular, self.max_angular)

        # --- COUPLING ---
        linear = linear * math.cos(angle)

        # --- COMMAND ---
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(-angular)  # flip sign if needed

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"dist={depth:.2f}m angle={math.degrees(angle):.1f}deg "
            f"lin={linear:.2f} ang={angular:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
