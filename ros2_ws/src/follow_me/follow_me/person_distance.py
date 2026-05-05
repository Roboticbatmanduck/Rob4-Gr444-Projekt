import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from follow_me_interfaces.msg import PersonBBox
from std_msgs.msg import Float32
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
       self.declare_parameter("distance_topic", "/distance/measured")

       self.declare_parameter("depth_scale", 1000.0)
       self.declare_parameter("histogram_bins", 64)
       self.declare_parameter("min_valid_depth_m", 0.20)
       self.declare_parameter("max_valid_depth_m", 6.00)
       self.declare_parameter("bbox_shrink_x", 0.2)
       self.declare_parameter("bbox_shrink_y", 0.2)
       self.declare_parameter("sync_queue_size", 10)
       self.declare_parameter("sync_slop", 0.05)
       self.declare_parameter("foreground_margin_m", 0.20)

       self.bbox_topic = self.get_parameter("bbox_topic").value
       self.depth_topic = self.get_parameter("depth_topic").value
       self.distance_topic = self.get_parameter("distance_topic").value

       self.depth_scale = float(self.get_parameter("depth_scale").value)
       self.histogram_bins = int(self.get_parameter("histogram_bins").value)
       self.min_valid_depth_m = float(self.get_parameter("min_valid_depth_m").value)
       self.max_valid_depth_m = float(self.get_parameter("max_valid_depth_m").value)
       self.bbox_shrink_x = float(self.get_parameter("bbox_shrink_x").value)
       self.bbox_shrink_y = float(self.get_parameter("bbox_shrink_y").value)
       self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
       self.sync_slop = float(self.get_parameter("sync_slop").value)
       self.foreground_margin_m = float(self.get_parameter("foreground_margin_m").value)

       self.bridge = CvBridge()

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
       
       self.sync = ApproximateTimeSynchronizer(
          [self.depth_sub, self.bbox_sub],
          queue_size=self.sync_queue_size,
          slop=self.sync_slop,
        )

       # Publisher
       self.publisher = self.create_publisher(
           Float32,
           self.distance_topic,
           10
       )

       self.sync.registerCallback(self.synced_callback)
       
       self.get_logger().info("DistanceNode started")

   def synced_callback(self, depth_msg, bbox_msg):
        if not bbox_msg.valid:
            return

        depth_image = self.bridge.imgmsg_to_cv2(
            depth_msg,
            desired_encoding="passthrough",
        )

        depth_crop = self.crop_depth_to_bbox(depth_image, bbox_msg)

        if depth_crop is None:
            return

        distance = self.estimate_distance(depth_crop)

        if distance is None:
            return

        msg = Float32()
        msg.data = float(distance)
        self.publisher.publish(msg)

        self.get_logger().debug(f"Published distance: {msg.data:.3f} m") 


   def crop_depth_to_bbox(self, depth_image, bbox_msg):
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
       
       return depth[y1:y2, x1:x2]

   def estimate_distance(self, depth_crop):
       values = depth_crop.flatten()
       values = values[np.isfinite(values)]
       values = values[values > 0]

       if values.size == 0:
           return None

       values_m = values / self.depth_scale

       values_m = values_m[(values_m >= self.min_valid_depth_m) 
                          & (values_m <= self.max_valid_depth_m)] 
       
       if values_m.size == 0:
           return None
       
       hist, bins = np.histogram(
           values_m, 
           bins=self.histogram_bins, 
           range=(self.min_valid_depth_m, self.max_valid_depth_m)
       )

       if np.max(hist) == 0:
           return None

       peak_index = np.argmax(hist)
       peak_center = (bins[peak_index] + bins[peak_index + 1]) / 2.0

       lower = peak_center - self.foreground_margin_m
       upper = peak_center + self.foreground_margin_m

       foreground_pixels = values_m[
           (values_m >= lower) & (values_m <= upper)
        ] 

       if foreground_pixels.size == 0:
           return None

       return float(np.mean(foreground_pixels))


def main(args=None):
   rclpy.init(args=args)


   node = DistanceNode()
   rclpy.spin(node)


   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

