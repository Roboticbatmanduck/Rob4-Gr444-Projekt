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

        self.setup()

        #Log the startup message and the subscribed and published topics
        self.get_logger().info(f"YoloPersonCenter node has been started. Subscribed to {self.image_topic} and publishing to {self.center_topic}.")

    def setup(self): #this function is called in the initialization of the node to set up parameters, the YOLO model, and the ROS subscriptions and publications.

        #Declare parameters for easy yaml configuration and command line overrides. 
        self.declare_parameter("image_topic", "/camera/camera/image_raw")
        self.declare_parameter("center_topic", "/person_center")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("lost_frame_limit", 10)

        #Get parameters
        self.image_topic = self.get_parameter("image_topic").value
        self.center_topic = self.get_parameter("center_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.lost_frame_limit = int(self.get_parameter("lost_frame_limit").value)

        #Define cv2 bridge and YOLO model
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect")

        #Remember values of the last detected person center and how many frames have lost detection in a row.
        self.last_center = None
        self.lost_frames = 0

        # Ros2 quality of service settings for the video stream and the person center topic
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        #Subscribe to the image topic and publish the person center
        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,                           #Whenever a new image is received, the image_callback function will be called
            qos,
        )

        self.pub = self.create_publisher(
            Point, 
            self.center_topic, 
            10,
        )

    def image_callback(self, msg):     
        #The image_callback function is called whenever a new image is received on the raw_image topic.
        # The function converts the incomming image to Opencv format
        # runs the YOLO model on the image to detect people and their confidence scores
        # selects the person closest to the previous centerpoint to maintain tracking of the same person across frames
        # publishes the center point of the detected person as a Point message on the person_center topic

        #Convert the ROS image message to an OpenCV image with bgr8 for the YOLO model
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        #Run the YOLO model on the image with the specified confidence threshold. Verbose=False will turn off clutter messages in the terminal. The result contains the detected bounding boxes and confidence scores.
        #If more than one person is detected, results will contain multiple detections, which will be sorted by the find_best_target function.  
        results = self.model(frame, verbose=False, conf=self.confidence_threshold)

        best_target = self.find_best_target(results)

        #if best_target is None:
        # count the amount if times no valid target is detected
        # if the amount is above the threshold then publish nothing and try again
        # This is to prevent the publishsing of old data and to make subsequent control code easier, as it can just check if no data is recieved for a certain amount of time to know that the target is lost
        # This also handles if the target is lost completely allowing for a new detection to be selected based on confidence scores rather than proximity.
        if best_target is None:
            self.handle_lost_target()
            return
        
        self.publish_target(best_target)

    def find_best_target(self, results):
        #This function takes the results from the YOLO model and finds the best target to track based on the confidence scores and proximity to the last detected center point.
            #if we have a previous target, we want to continue tracking it.
            # if we don't have a previous target, we want to select the one with the highest confidence score. 


        best_target = None

        if self.last_center is None:
            best_value = float("-inf") #we want to make sure that all detections are better than the initial value, so we set it to negative infinity for the case where we don't have a previous target.
        else:
            best_value = float("inf") #if we have a previous target, we want to ignore confidence scores and only look at the distance to the last center. Therefore, the initial value is set to positive infinity.

        for result in results:
            for box in result.boxes:
                confidence = float(box.conf[0]) #confidence score of the detection

                if confidence < self.confidence_threshold:
                    continue #ignore detections below the confidence threshold

                x1, y1, x2, y2 = box.xyxy[0].tolist() #bounding box coordinates

                #compute center of bounding box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Save center point and confidence score of this detection
                target = (center_x, center_y, confidence)

                if self.last_center is None:
                    #if we don't have a previous target, we want to select the one with the highest confidence score.
                    #if this detections confidence is higher than the best so far, we update the best target and best value.
                    value = confidence

                    if value > best_value:
                        best_value = value
                        best_target = target
                
                else:
                    #a person is already tracked, so we want to continue tracking them. We ignore confidence scores and look at the distance from this detection to the last published detection.
                    value = self.distance_from_last_center(center_x, center_y)
                    if value < best_value:
                        best_value = value
                        best_target = target
        return best_target  

    def distance_from_last_center(self, center_x, center_y):
        #This function computes the distance to the last published center in pixels
        
        #compute the difference in x and y coordinates between the current detection and the last published center
        dx = center_x - self.last_center[0]
        dy = center_y - self.last_center[1]

        #compute the Euclidean distance using the Pythagorean theorem and return it (**0.5 is equivalent to the square root)
        return (dx * dx + dy * dy) ** 0.5
    
    def handle_lost_target(self):
        #This function is called when no valid person is detected in the current frame
        #it counts the amount of consecutive frames where no valid target is detected
        # if the amount of lost frames exceeds the specified limit, the last center is reset to None, allowing the next detection to be selected based on confidence scores rather than proximity. 

        self.lost_frames += 1

        if self.lost_frames >= self.lost_frame_limit:
            self.last_center = None

    def publish_target(self, target):
        #This function publishes the detected target as a Point message on the person_center topic and resets the lost frame count.

        center_x, center_y, confidence = target

        #Update last center and reset lost frame count, days since last incident to zero ;D
        self.last_center = (center_x, center_y)
        self.lost_frames = 0

        point = Point()
        point.x = float(center_x)
        point.y = float(center_y)
        point.z = float(confidence)

        self.pub.publish(point)

#typical ROS2 Python node main function, which initializes the node, spins it to keep it alive and handle shutdown gracefully when needed.
def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonCenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
