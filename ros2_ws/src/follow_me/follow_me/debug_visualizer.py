import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from follow_me_interfaces.msg import PersonBBox

from cv_bridge import CvBridge
import cv2


class DebugVisualizer(Node):
    def __init__(self):
        super().__init__("debug_visualizer")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("bbox_topic", "/person_bbox")
        self.declare_parameter("distance_topic", "/distance/measured")
        self.declare_parameter("angle_topic", "/angle/measured")
        self.declare_parameter("debug_image_topic", "/follow_me/debug_image")

        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.distance_topic = self.get_parameter("distance_topic").value
        self.angle_topic = self.get_parameter("angle_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        self.bridge = CvBridge()

        self.latest_bbox = None
        self.latest_distance = None
        self.latest_angle = None

        self.create_subscription(follow
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.create_subscription(
            PersonBBox,
            self.bbox_topic,
            self.bbox_callback,
            10,
        )

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

        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            10,
        )

        self.get_logger().info("Debug visualizer started")

    def bbox_callback(self, msg):
        self.latest_bbox = msg

    def distance_callback(self, msg):
        self.latest_distance = float(msg.data)

    def angle_callback(self, msg):
        self.latest_angle = float(msg.data)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        self.draw_bbox(frame)
        self.draw_text_info(frame)

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug_msg.header = msg.header

        self.debug_image_pub.publish(debug_msg)

    def draw_bbox(self, frame):
        if self.latest_bbox is None:
            return

        if not self.latest_bbox.valid:
            return

        height, width = frame.shape[:2]

        x1 = int(max(0, min(self.latest_bbox.x1, width - 1)))
        y1 = int(max(0, min(self.latest_bbox.y1, height - 1)))
        x2 = int(max(0, min(self.latest_bbox.x2, width - 1)))
        y2 = int(max(0, min(self.latest_bbox.y2, height - 1)))

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        confidence_text = f"conf: {self.latest_bbox.confidence:.2f}"

        cv2.putText(
            frame,
            confidence_text,
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    def draw_text_info(self, frame):
        y = 30
        line_spacing = 30

        if self.latest_distance is None:
            distance_text = "distance: --- m"
        else:
            distance_text = f"distance: {self.latest_distance:.2f} m"

        if self.latest_angle is None:
            angle_text = "angle: --- deg"
        else:
            angle_text = f"angle: {self.latest_angle:.2f} deg"

        cv2.putText(
            frame,
            distance_text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            frame,
            angle_text,
            (20, y + line_spacing),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DebugVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()