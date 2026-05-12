import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, Bool
from follow_me_interfaces.msg import PersonBBox
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
import numpy as np
import cv2


class DebugVisualizer(Node):
    def __init__(self):
        super().__init__("debug_visualizer")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("bbox_topic", "/person_bbox")
        self.declare_parameter("distance_topic", "/distance/measured")
        self.declare_parameter("angle_topic", "/angle/measured")
        self.declare_parameter("debug_image_topic", "/follow_me/debug/compressed")
        self.declare_parameter("distance_point_topic", "/distance/point")
        self.declare_parameter("detect_topic", "/detect")
        self.declare_parameter("angle_deg_measured", "/angle/measured/deg")
        
        self.declare_parameter("bbox_shrink_x", 0.1)
        self.declare_parameter("bbox_shrink_y", 0.1)

        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.distance_topic = self.get_parameter("distance_topic").value
        self.angle_topic = self.get_parameter("angle_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.distance_point_topic = self.get_parameter("distance_point_topic").value
        self.bbox_shrink_x = float(self.get_parameter("bbox_shrink_x").value)
        self.bbox_shrink_y = float(self.get_parameter("bbox_shrink_y").value)
        self.detect_topic = self.get_parameter("detect_topic").value
        self.angle_deg = self.get_parameter("angle_deg_measured").value

        self.bridge = CvBridge()

        self.latest_bbox = None
        self.latest_distance = None
        self.latest_angle = None

        self.latest_distance_angle = None
        self.latest_distance_point = None

        self.detect = False

        self.create_subscription(
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

        self.create_subscription(
            PointStamped,
            self.distance_point_topic,
            self.distance_point_callback,
            10,
        )

        self.debug_image_pub = self.create_publisher(
            CompressedImage,
            self.debug_image_topic,
            10,
        )
        self.detect_publisher = self.create_publisher(
            Bool,
            self.detect_topic,
            10
        )
        self.degree_publisher = self.create_publisher(
            Float32,
            self.angle_deg,
            10
        )

        self.get_logger().info("Debug visualizer started")

    def bbox_callback(self, msg):
        self.latest_bbox = msg
        self.detect = self.latest_bbox.valid

    def distance_callback(self, msg):
        self.latest_distance = float(msg.data)

    def angle_callback(self, msg):
        self.latest_angle = float(msg.data)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        self.draw_bbox(frame)
        self.draw_text_info(frame)

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        debug_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
        comp_msg = CompressedImage()
        comp_msg.header = msg.header
        comp_msg.format = "jpeg"

        success, encoded = cv2.imencode(".jpg", frame)
        if not success:
            return

        comp_msg.data = encoded.tobytes()

        self.debug_image_pub.publish(comp_msg)
        detect_msg = Bool()
        detect_msg.data = self.detect
        self.detect_publisher.publish(detect_msg)
        deg_msg = Float32()
        if self.latest_angle is not None:
            deg_msg.data = float(np.rad2deg(self.latest_angle))
        else:
            deg_msg.data = 0.0  # eller skip entirely
        self.degree_publisher.publish(deg_msg)

    def distance_point_callback(self, msg):
        self.latest_distance_point = msg

    def draw_bbox(self, frame):
        if self.latest_bbox is None:
            return

        if not self.latest_bbox.valid:
            return

        height, width = frame.shape[:2]

        x1 = int(max(0, min(self.latest_bbox.x1, width)))
        y1 = int(max(0, min(self.latest_bbox.y1, height)))
        x2 = int(max(0, min(self.latest_bbox.x2, width)))
        y2 = int(max(0, min(self.latest_bbox.y2, height)))

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        box_w = x2 - x1
        box_h = y2 - y1

        sx = self.bbox_shrink_x
        sy = self.bbox_shrink_y

        inner_x1 = int(x1 + sx * box_w)
        inner_x2 = int(x2 - sx * box_w)
        inner_y1 = int(y1 + sy * box_h)
        inner_y2 = int(y2 - sy * box_h)

        cv2.rectangle(
            frame,
            (inner_x1, inner_y1),
            (inner_x2, inner_y2),
            (255, 0, 0),
            2,
        )

        # Center point of bounding box
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        cv2.circle(
            frame,
            (center_x, center_y),
            4,
            (0, 0, 255),
            -1,
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

        # Display the latest distance point if available
        if self.latest_distance_point is not None:
            d_point_x = int(self.latest_distance_point.point.x)
            d_point_y = int(self.latest_distance_point.point.y)

            if 0 <= d_point_x < width and 0 <= d_point_y < height:
                cv2.circle(
                    frame,
                    (d_point_x, d_point_y),
                    6,
                    (255, 255, 0),
                    -1,
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
            angle_text = f"angle: {np.rad2deg(self.latest_angle):.2f} deg"

        if self.latest_distance is None:
            camera_distance = "Hypotenuse: ---"
        else:
            camera_distance = f"Hypotenuse: {self.latest_distance_point.point.z:.2f} m"

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

        cv2.putText(
            frame,
            camera_distance,
            (20, y + 2 * line_spacing),
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