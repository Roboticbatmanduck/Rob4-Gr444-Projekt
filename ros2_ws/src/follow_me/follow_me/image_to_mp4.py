import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import cv2
import numpy as np
import csv


class CompressedVideoRecorder(Node):
    def __init__(self):
        super().__init__("compressed_video_recorder")

        self.declare_parameter("image_topic", "/follow_me/debug/compressed")
        self.declare_parameter("detect_topic", "/detect")
        self.declare_parameter("output_path", "output.mp4")
        self.declare_parameter("csv_path", "output.csv")
        self.declare_parameter("fps", 30.0)

        self.topic = self.get_parameter("image_topic").value
        self.detect_topic = self.get_parameter("detect_topic").value
        self.output_path = self.get_parameter("output_path").value
        self.csv_path = self.get_parameter("csv_path").value
        self.fps = float(self.get_parameter("fps").value)

        self.video_writer = None
        self.latest_detect = False
        self.frame_id = 0

        # CSV
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["frame", "detect"])

        # Subscribers
        self.create_subscription(
            CompressedImage,
            self.topic,
            self.image_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.detect_topic,
            self.detect_callback,
            10
        )

        self.get_logger().info(f"Recording video → {self.output_path}")
        self.get_logger().info(f"Recording CSV → {self.csv_path}")

    def detect_callback(self, msg):
        self.latest_detect = msg.data

    def image_callback(self, msg):
        # Decode image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        h, w, _ = frame.shape

        # Init video writer
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(
                self.output_path,
                fourcc,
                self.fps,
                (w, h)
            )
            self.get_logger().info(f"Video initialized: {w}x{h}")

        # Write video frame
        self.video_writer.write(frame)

        # Write CSV (1 row per frame)
        self.csv_writer.writerow([self.frame_id, int(self.latest_detect)])

        self.frame_id += 1

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("Video saved")

        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info("CSV saved")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CompressedVideoRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
