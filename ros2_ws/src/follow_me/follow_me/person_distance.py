import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from follow_me_interfaces.msg import PersonBBox
from std_msgs.msg import Float32

from cv_bridge import CvBridge
import numpy as np


class DistanceNode(Node):

    def __init__(self):
        super().__init__('afstand_node')

        self.bridge = CvBridge()

        self.depth_image = None
        self.target_pixel = None

        self.image_width = 640
        self.image_height = 480

        # Subscribers
        self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        self.create_subscription(
            Point,
            '/target_pixel',
            self.point_callback,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(
            Float32,
            '/afstand',
            10
        )

    def depth_callback(self, msg):
        print("DEPTH CALLBACK")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.compute_distance()

    def point_callback(self, msg):
        print("POINT CALLBACK")
        self.target_pixel = (int(msg.x), int(msg.y))
        self.compute_distance()

    def mean_distance(self, cx, cy, kernel_size=3):
        if self.depth_image is None:
            return None

        half = kernel_size // 2
        values = []

        for i in range(-half, half + 1):
            for j in range(-half, half + 1):

                x = int(np.clip(cx + i, 0, self.image_width - 1))
                y = int(np.clip(cy + j, 0, self.image_height - 1))

                d = self.depth_image[y, x]

                # Convert if needed (RealSense often uses mm)
                if d > 0:
                    values.append(d)

        if len(values) == 0:
            return None

        return np.mean(values)

    def compute_distance(self):
        if self.depth_image is None or self.target_pixel is None:
            return

        u, v = self.target_pixel

        dist = self.mean_distance(u, v)

        if dist is None:
            self.get_logger().warn("No valid depth")
            return

        # Convert mm → meters if needed
        if dist > 10:  # crude check
            dist = dist / 1000.0

        msg = Float32()
        msg.data = float(np.round(dist, 3))

        self.publisher.publish(msg)

        self.get_logger().info(f"Distance: {msg.data:.3f} m")


def main(args=None):
    rclpy.init(args=args)

    node = DistanceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()