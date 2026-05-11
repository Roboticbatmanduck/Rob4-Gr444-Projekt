import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import numpy as np
import math


class SimpleFollower(Node):
    def __init__(self):
        super().__init__("simple_follower")

        # Parameters
        self.declare_parameter("distance_topic", "/distance/measured")
        self.declare_parameter("angle_topic", "/angle/measured")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("target_distance", 1.5)
        self.declare_parameter("max_linear", 0.2)
        self.declare_parameter("max_angular", 0.2)

        self.declare_parameter("linear_gain", 0.5)
        self.declare_parameter("angular_gain", 1.0)

        self.distance_topic = self.get_parameter("distance_topic").value
        self.angle_topic = self.get_parameter("angle_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.target_distance = float(self.get_parameter("target_distance").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)

        self.latest_distance = None
        self.latest_angle = None
        self.last_msg_time = self.get_clock().now()

        self.timeout = 0.5

        self.create_subscription(
            Float32,
            self.distance_topic,
            self.distance_callback,
            10,
        )

        self.create_subscription(
            Float32,
            self.angle_topic,
            self.angle_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Simple follower started. Subscribed to {self.distance_topic} and {self.angle_topic}"
        )

    def distance_callback(self, msg):
        self.latest_distance = float(msg.data)
        self.last_msg_time = self.get_clock().now()

    def angle_callback(self, msg):
        self.latest_angle = float(msg.data)
        self.last_msg_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_msg_time).nanoseconds * 1e-9

        if self.latest_distance is None or self.latest_angle is None or dt > self.timeout:
            self.publish_stop()
            return

        distance = self.latest_distance
        angle = self.latest_angle

        # If angle node publishes degrees, convert to radians.
        # If it already publishes radians, delete these two lines.
        if abs(angle) > math.pi:
            angle = math.radians(angle)

        distance_error = distance - self.target_distance

        linear = self.linear_gain * distance_error
        angular = self.angular_gain * angle

        linear = float(np.clip(linear, -self.max_linear, self.max_linear))
        angular = float(np.clip(angular, -self.max_angular, self.max_angular))

        # Slow forward motion while turning
        linear *= max(0.0, math.cos(angle))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = -angular  # flip sign if turning wrong way

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"distance={distance:.2f}m angle={math.degrees(angle):.1f}deg "
            f"linear={cmd.linear.x:.2f} angular={cmd.angular.z:.2f}"
        )

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()