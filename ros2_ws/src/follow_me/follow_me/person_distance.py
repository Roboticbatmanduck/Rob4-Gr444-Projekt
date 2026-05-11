import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from follow_me_interfaces.msg import PersonBBox

from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
import numpy as np




class DistanceNode(Node):


   def __init__(self):
       super().__init__('person_distance')

       # Parameters
       self.declare_parameter("bbox_topic", "/person_bbox")
       self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
       self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

       self.declare_parameter("distance_topic", "/distance/measured")
       self.declare_parameter("distance_point_topic", "/distance/point")

       self.declare_parameter("depth_scale", 1000.0)
       self.declare_parameter("histogram_bins", 64)
       self.declare_parameter("min_valid_depth_m", 0.20)
       self.declare_parameter("max_valid_depth_m", 5.00)
       self.declare_parameter("bbox_shrink_x", 0.2)
       self.declare_parameter("bbox_shrink_y", 0.2)
       self.declare_parameter("sync_queue_size", 10)
       self.declare_parameter("sync_slop", 0.05)
       self.declare_parameter("foreground_margin_m", 0.20)
       self.declare_parameter("camera_pitch_deg", 20.0)

       self.bbox_topic = self.get_parameter("bbox_topic").value
       self.depth_topic = self.get_parameter("depth_topic").value
       self.camera_info_topic = self.get_parameter("camera_info_topic").value
       self.distance_topic = self.get_parameter("distance_topic").value
       self.distance_point_topic = self.get_parameter("distance_point_topic").value
       self.depth_scale = float(self.get_parameter("depth_scale").value)
       self.histogram_bins = int(self.get_parameter("histogram_bins").value)
       self.min_valid_depth_m = float(self.get_parameter("min_valid_depth_m").value)
       self.max_valid_depth_m = float(self.get_parameter("max_valid_depth_m").value)
       self.bbox_shrink_x = float(self.get_parameter("bbox_shrink_x").value)
       self.bbox_shrink_y = float(self.get_parameter("bbox_shrink_y").value)
       self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
       self.sync_slop = float(self.get_parameter("sync_slop").value)
       self.foreground_margin_m = float(self.get_parameter("foreground_margin_m").value)
       self.camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
       
       self.bridge = CvBridge()

       #camera intrinsics
       self.fx = None
       self.fy = None
       self.cx = None
       self.cy = None  

       #quality of service settings for the depth and bbox topics to ensure we get the most recent messages and can handle some delay between them.
       sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        ) 

       # Subscribers
       self.depth_sub = Subscriber(
          self,
          Image,
          self.depth_topic,
          qos_profile=sensor_qos,
        )
       
       self.bbox_sub = Subscriber(
          self,
          PersonBBox,
          self.bbox_topic,
          qos_profile=10,
        )
       
       self.camera_info_sub = self.create_subscription(
          CameraInfo,
          self.camera_info_topic,
          self.camera_info_callback,
          sensor_qos,
        )       
       
       self.sync = ApproximateTimeSynchronizer(
          [self.depth_sub, self.bbox_sub],
          queue_size=self.sync_queue_size,
          slop=self.sync_slop,
        )

       # Publisher
       self.distance_publisher = self.create_publisher(
           Float32,
           self.distance_topic,
           10,
       )

       self.point_publisher = self.create_publisher(
           PointStamped,
           "/distance/point",
           10,
       )

       self.sync.registerCallback(self.synced_callback)
       
       self.get_logger().info("DistanceNode started")

   def camera_info_callback(self, msg):
       """
        Stores the camera intrinsic parameters.

        Camera matrix K:
            [fx  0 cx]
            [ 0 fy cy]
            [ 0  0  1]
        """
       if self.fx is not None:
           return

       self.fx = msg.k[0]
       self.fy = msg.k[4]
       self.cx = msg.k[2]
       self.cy = msg.k[5]

       self.get_logger().debug(
              f"Received camera intrinsics: "
              f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
              f"cx={self.cx:.2f}, cy={self.cy:.2f}"
        )

   def synced_callback(self, depth_msg, bbox_msg):
        #called when we have a new pair of depth and bbox messages that are close enough in time. 
        if not bbox_msg.valid:
            return

        if self.fx is None:
            self.get_logger().debug("Camera intrinsics not received yet, cannot compute distance")
            return

        depth_image = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding="passthrough",
        )

        crop_result = self.crop_depth_to_bbox(depth_image, bbox_msg)

        if crop_result is None:
            return

        depth_crop, x_offset, y_offset = crop_result

        result = self.estimate_distance_and_point(
            depth_crop,
            x_offset,
            y_offset,
        )

        if result is None:
            return
        
        distance, mean_z, mean_u, mean_v = result

        #Publish point and distance
        distance_msg = Float32()
        distance_msg.data = float(distance)
        self.distance_publisher.publish(distance_msg)

        point_msg = PointStamped()
        point_msg.header.stamp = depth_msg.header.stamp
        point_msg.header.frame_id = depth_msg.header.frame_id
 
        point_msg.point.x = float(mean_u)
        point_msg.point.y = float(mean_v)
        point_msg.point.z = float(mean_z)

        self.point_publisher.publish(point_msg)

        self.get_logger().debug(
            f"Published distance: {distance_msg.data:.3f} m, "
            f"frame_point: ({mean_u:.3f}, {mean_v:.3f}, {mean_z:.3f})"
            ) 


   def crop_depth_to_bbox(self, depth_image, bbox_msg):
       #Crops the depth image to the YOLO bounding box
       #The crop is further shrunk by a percentage to avoid including too much background

       depth = np.array(depth_image, dtype=np.float32)

       height, width = depth.shape[:2]
 
       x1 = int(np.clip(bbox_msg.x1, 0, width))
       y1 = int(np.clip(bbox_msg.y1, 0, height))
       x2 = int(np.clip(bbox_msg.x2, 0, width))
       y2 = int(np.clip(bbox_msg.y2, 0, height))

       # Crop to the bounding box to focus on the person and reduce background noise 
       box_width = x2 - x1
       box_height = y2 - y1

       x1 += int(self.bbox_shrink_x * box_width)
       x2 -= int(self.bbox_shrink_x * box_width)
       y1 += int(self.bbox_shrink_y * box_height)
       y2 -= int(self.bbox_shrink_y * box_height)

       if x2 <= x1 or y2 <= y1:
            return None 
       
       depth_crop = depth[y1:y2, x1:x2]

       #return the cropped depth image
       # also return the x and y offsets of the crop relative to the original image so we can map points back to the original coordinate frame later 
       return depth_crop, x1, y1

   def estimate_distance_and_point(self, depth_crop, x_offset, y_offset):
       #Estimates the distance to the person

       #1. Convert depth crop to meters
       #2. Remove invalid depth values
       #3. Use historgram to find the dominant depth peak
       #4. Select pixels close to that peak as foreground
       #5. Back-project those pixels to 3D camera coords
       #6. Average the 3D points to get a stable distance estimate and point
       #7. Project the averaged point onto the horizontal ground plane using the known camera pitch angle

       depth_m = depth_crop.astype(np.float32) / self.depth_scale
       
       valid_mask = (
           np.isfinite(depth_m) &
           (depth_m > 0) &
           (depth_m >= self.min_valid_depth_m) &
           (depth_m <= self.max_valid_depth_m)
       )

       values_m = depth_m[valid_mask]

       if values_m.size == 0:
           return None
       
       hist, bins = np.histogram(
           values_m, 
           bins=self.histogram_bins, 
           range=(self.min_valid_depth_m, self.max_valid_depth_m)
       )

       if np.max(hist) == 0:
           return None
       
       peak_index = int(np.argmax(hist))
       peak_center = (bins[peak_index] + bins[peak_index + 1]) / 2.0

       lower = peak_center - self.foreground_margin_m
       upper = peak_center + self.foreground_margin_m

       foreground_mask = (
           valid_mask
           & (depth_m >= lower)
           & (depth_m <= upper)
       )

       if np.count_nonzero(foreground_mask) == 0:
           return None

       #pixel coords inside the cropped region 
       v_crop, u_crop = np.where(foreground_mask)

       #depth values of those pixels 
       z = depth_m[v_crop, u_crop]

       #convert to pixel coords in the original image 
       u = u_crop + x_offset
       v = v_crop + y_offset

       #back-project to 3D camera coords
       x = (u - self.cx) * z / self.fx
       y = (v - self.cy) * z / self.fy

       mean_x = float(np.mean(x)) 
       mean_y = float(np.mean(y)) 
       mean_z = float(np.mean(z))

       mean_u = float(np.mean(u))
       mean_v = float(np.mean(v))  

       #correct for camera pitch to get horizontal distance on the ground plane
       pitch_rad = math.radians(self.camera_pitch_deg)

       forward_horisontal = (
           mean_z * math.cos(pitch_rad) 
           + mean_y * math.sin(pitch_rad)
       ) 

       side_horizontal = mean_x

       horizontal_distance = math.sqrt(
           forward_horisontal**2 + side_horizontal**2
       )
       

       return horizontal_distance, mean_z, mean_u, mean_v


def main(args=None):
   rclpy.init(args=args)


   node = DistanceNode()
   try:
        rclpy.spin(node)
   except KeyboardInterrupt:
        pass
   finally:  
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
   main()

