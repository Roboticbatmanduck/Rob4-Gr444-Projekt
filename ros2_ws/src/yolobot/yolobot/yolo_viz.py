import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetectorViz(Node):
    def __init__(self):
        super().__init__('yolo_detector_viz')

        self.bridge = CvBridge()

        # Use YOLO nano
        self.model = YOLO("yolov8n.pt")

        # Your Gazebo camera topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.process_image,
            10
        )

        self.viz_pub = self.create_publisher(
            Image,
            '/yolo/visualization',
            10
        )

        self.get_logger().info("YOLOv8 nano detector started on /camera/image_raw")

    def process_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Only detect humans: COCO class 0 = person
        results = self.model(frame, conf=0.25, imgsz=640, verbose=False)

        annotated = results[0].plot()

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.viz_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
