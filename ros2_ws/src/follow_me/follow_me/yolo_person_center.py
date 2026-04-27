import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ultralytics import YOLO


class YoloPersonCenter(Node):
    def __init__(self):
        super().__init__("yolo_person_center")

        self.bridge = CvBridge()
        self.model = YOLO("/workspace/src/follow_me/engine/best.engine")

        # Low-latency QoS (important for camera)
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            self.qos,
        )

        self.pub = self.create_publisher(
            Point,
            "/person_center",
            10,
        )

        self.get_logger().info("YOLO person center node started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(frame, verbose=False)

        best_person = None
        best_conf = 0.0

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])

                if cls == 0 and conf > best_conf:
                    best_conf = conf
                    best_person = box

        if best_person is None:
            return

        x1, y1, x2, y2 = best_person.xyxy[0].tolist()

        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0

        point = Point()
        point.x = float(center_x)
        point.y = float(center_y)
        point.z = float(best_conf)

        self.pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonCenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
