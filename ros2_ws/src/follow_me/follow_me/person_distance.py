import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image
from follow_me_interfaces.msg import PersonBBox
from std_msgs.msg import Float32


from cv_bridge import CvBridge
import numpy as np




class DistanceNode(Node):


   def __init__(self):
       super().__init__('afstand_node')


       self.bridge = CvBridge()


       self.depth_image = None
       self.target_pixel = None


       self.image_width = 640
       self.image_height = 480


       self.crop = None  # (x1, y1, x2, y2)


       # Subscribers
       self.create_subscription(
           Image,
           '/camera/depth/image_rect_raw',
           self.depth_callback,
           10
       )


       self.create_subscription(
           PersonBBox,
           '/bbox',
           self.bbox_callback,
           10
       )


       # Publisher
       self.publisher = self.create_publisher(
           Float32,
           '/afstand',
           10
       )




   def bbox_callback(self, msg):
       x1, y1, x2, y2 = msg.data


       self.crop = (
           int(x1),
           int(y1),
           int(x2),
           int(y2)
       )


   def depth_callback(self, msg):
       if self.crop is None:
           return


       range_min = 0.0
       range_max = 5.0   


       depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
       depth = np.array(depth, dtype=np.float32)


       x1, y1, x2, y2 = self.crop


       # Clamp to image bounds
       h, w = depth.shape
       x1, x2 = np.clip([x1, x2], 0, w)
       y1, y2 = np.clip([y1, y2], 0, h)


       roi = depth[y1:y2, x1:x2]


       values = roi.flatten()
       values = values[np.isfinite(values)]
       values = values[values > 0]


       if values.size == 0:
           return


       values_m = values / 1000.0


       # the range of 0-5    
       hist, bins = np.histogram(values_m, bins=50, range=(range_min, range_max))
  


       peak_index = np.argmax(hist)
       distance = (bins[peak_index] + bins[peak_index + 1]) / 2


       msg = Float32()
       msg.data = float(distance)
       self.publisher.publish(msg)


       self.get_logger().info(f"Distance: {distance:.2f} m")




def main(args=None):
   rclpy.init(args=args)


   node = DistanceNode()
   rclpy.spin(node)


   node.destroy_node()
   rclpy.shutdown()




if __name__ == '__main__':
   main()

