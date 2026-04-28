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

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("center_topic", "/person_center")
        self.declare_parameter("model_path", "/workspace/src/follow_me/engine/best.engine")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("distance_penalty", 0.002)
        self.declare_parameter("lost_frame_limit", 10)

        self.image_topic = self.get_parameter("image_topic").value
        self.center_topic = self.get_parameter("center_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.distance_penalty = float(self.get_parameter("distance_penalty").value)
        self.lost_frame_limit = int(self.get_parameter("lost_frame_limit").value)

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect")

        self.last_center = None
        self.lost_frames = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos,
        )

        self.pub = self.create_publisher(
            Point,
            self.center_topic,
            10,
        )

        self.get_logger().info("YOLO person center node started")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Center topic: {self.center_topic}")
        self.get_logger().info(f"Model path: {self.model_path}")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model.predict(frame, verbose=False, task="detect")

        best_target = None
        best_score = -1e9

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])

                # YOLO class 0 = person
                if cls != 0:
                    continue

                if conf < self.confidence_threshold:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0

                if self.last_center is not None:
                    dx = center_x - self.last_center[0]
                    dy = center_y - self.last_center[1]
                    distance = (dx * dx + dy * dy) ** 0.5
                else:
                    distance = 0.0

                score = conf - self.distance_penalty * distance

                if score > best_score:
                    best_score = score
                    best_target = (center_x, center_y, conf)

        if best_target is None:
            self.lost_frames += 1

            if self.lost_frames >= self.lost_frame_limit:
                self.last_center = None

            return

        center_x, center_y, conf = best_target

        self.last_center = (center_x, center_y)
        self.lost_frames = 0

        point = Point()
        point.x = float(center_x)
        point.y = float(center_y)
        point.z = float(conf)

        self.pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonCenter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
