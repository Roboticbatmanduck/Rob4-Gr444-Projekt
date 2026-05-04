import rclpy
from rclpy.node import Node

from follow_me_interfaces.msg import PersonBBox
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32

import math


class PixelToAngle(Node):
    def __init__(self):
        super().__init__("pixel_to_angle")

        self.declare_parameter("bbox_topic", "/person_bbox")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("angle_topic", "/angle/measured")

        # Topics
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.angle_topic = self.get_parameter("angle_topic").value

        # Camera parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # distortion coefficients
        self.k1 = 0.0
        self.k2 = 0.0
        self.k3 = 0.0
        self.p1 = 0.0
        self.p2 = 0.0

        # Subscribers
        self.pixel_sub = self.create_subscription(
            PersonBBox,
            self.bbox_topic,
            self.bbox_callback,
            10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        # Publisher
        self.angle_pub = self.create_publisher(
            Float32,
            self.angle_topic,
            10,
        )

        self.get_logger().info("PixelToAngle node started")


    # CameraInfo callback
    def camera_info_callback(self, msg: CameraInfo):

        if self.fx is not None:
            return
        
        # Intrinsics
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        # Distortion (robust parsing)
        d = msg.d

        self.k1 = d[0] if len(d) > 0 else 0.0
        self.k2 = d[1] if len(d) > 1 else 0.0
        self.p1 = d[2] if len(d) > 2 else 0.0
        self.p2 = d[3] if len(d) > 3 else 0.0
        self.k3 = d[4] if len(d) > 4 else 0.0

        self.get_logger().info(f"Camera intrinsics loaded: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}, k1={self.k1}, k2={self.k2}, p1={self.p1}, p2={self.p2} and k3={self.k3}")



    # BBox callback
    def bbox_callback(self, msg: PersonBBox):

        # Wait until camera is ready
        if self.fx is None:
            return
        
        if not msg.valid:
            self.get_logger().debug("Invalid BBox received, skipping")
            return

        u = (msg.x1 + msg.x2) / 2
        v = (msg.y1 + msg.y2) / 2

        self.get_logger().debug(f"BBox center recived: x={u} and y= {v}")

        # 1. Normalize pixel
        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy

        # 2. Undistort
        x_u, y_u = self.undistort(x, y)

        # 3. Angle (radians)
        theta = math.atan(x_u)

        # 4. Convert to degrees
        theta_deg = math.degrees(theta)

        self.get_logger().debug(f"Calculated angle={theta_deg}")

        # 5. Publish
        out = Float32()
        out.data = float(theta_deg)
        self.angle_pub.publish(out)


    # Undistortion
    def undistort(self, x, y):

        x_u = x
        y_u = y

        for _ in range(8):

            r2 = x_u * x_u + y_u * y_u
            r4 = r2 * r2
            r6 = r4 * r2

            # radial distortion
            radial = 1 + self.k1 * r2 + self.k2 * r4 + self.k3 * r6

            # tangential distortion
            x_tang = 2 * self.p1 * x_u * y_u + self.p2 * (r2 + 2 * x_u * x_u)
            y_tang = self.p1 * (r2 + 2 * y_u * y_u) + 2 * self.p2 * x_u * y_u

            # correction step
            x_u = (x - x_tang) / radial
            y_u = (y - y_tang) / radial

        return x_u, y_u



# Main
def main(args=None):
    rclpy.init(args=args)
    node = PixelToAngle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()