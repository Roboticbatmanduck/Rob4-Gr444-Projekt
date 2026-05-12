import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from follow_me_interfaces.msg import PersonBBox
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ultralytics import YOLO


class YoloPersonCenter(Node):
    def __init__(self):
        super().__init__("yolo_person_center")

        self.setup()

        #Print the startup message to the termnal to confirm that the node is running and to provide feedback to the user.
        self.get_logger().info(f"YoloPersonCenter node is started.")

    def setup(self): #this function is called in the initialization of the node to set up parameters, the YOLO model, and the ROS subscriptions and publications.

        #Declare parameters for easy yaml configuration and command line overrides. 
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("bbox_topic", "/person_bbox")
        self.declare_parameter("model_path", "/workspace/src/follow_me/engine/best.engine")
        self.declare_parameter("confidence_threshold", 0.7)
        self.declare_parameter("lost_frame_limit", 2)

        #Get parameters
        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.lost_frame_limit = int(self.get_parameter("lost_frame_limit").value)
        

        #Define cv2 bridge and YOLO model
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path, task="detect") #task="detect" is not strictly necessary as it is the default, unless another model is used.

        #Initialize values of the last detected person center and how many frames have lost detection in a row.
        self.last_center = None
        self.lost_frames = 0

        # Ros2 quality of service settings for the video stream and the person center topic
        #ReliabilityPolicy.BEST_EFFORT is used for the video stream to prioritize low latency over guaranteed delivery.
        #HistoryPolicy.KEEP_LAST with a depth of 1 is used for the person center topic to ensure that only the most recent detection is kept.
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
        #Publisher for the bounding box of the detected person. The PersonBBox message contains the coordinates of the bounding box, the confidence score, and a validity flag to indicate whether the detection is valid or not.
        self.pub = self.create_publisher(
            PersonBBox, 
            self.bbox_topic, 
            10,
        )


    def image_callback(self, msg):     
        '''
        The image_callback function is called whenever a new image is received on the raw_image topic.
        The function converts the incomming image to Opencv format
        runs the YOLO model on the image to detect people and their confidence scores
        First it picks the person with the highest confidence score to start tracking.
        Then it continues to track that person based on proximity to the last detected center point, ignoring confidence scores.
        if no valid target is detected for a certain amount of frames, it will publish an invalid bbox and reset the last center point to allow for a new target to be selected based on confidence score. 
        publishes the center point of the detected person as a PersonBBox message on the person_bbox topic
        '''

        #Convert the ROS image message to an OpenCV image with bgr8 for the YOLO model
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        #Run the YOLO model on the image with the specified confidence threshold. Verbose=False will turn off clutter messages in the terminal. The result contains the detected bounding boxes and confidence scores.
        #If more than one person is detected, results will contain multiple detections, which will be sorted by the find_best_target function.  
        results = self.model(frame, verbose=False)

        #Run the function to find the best target to track based on the results from the YOLO model.
        best_target = self.find_best_target(results)

        '''
        if best_target is None:
         count the amount if times no valid target is detected, via the handle_lost_target function.
         if the amount is above the threshold then reset the last center point and publish an invalid bbox (valid=false).
         This is to prevent the publishsing of old data and to make subsequent control code easier.
         This also handles if the target is lost completely allowing for a new detection to be selected based on confidence scores rather than proximity.

         If a valid target is detected, publish the target as a PersonBBox message on the person_bbox topic with valid=true and reset the lost frame count.
        '''
        if best_target is None:
            self.handle_lost_target(msg.header)
            return
        self.publish_valid_target(best_target, msg.header)
        

    def find_best_target(self, results):
        '''
        This function takes the results from the YOLO model and finds the best target.
        First it finds the best target based on highest confidence score.
        Then it continues to track that target based on proximity to the last detected center point, ignoring confidence scores. 
        '''

        #initialize local variable to store the best target as a tuple.
        best_target = None

        if self.last_center is None:
            best_value = float("-inf") #we want to make sure that all detections are better than the initial value, so we set it to negative infinity for the case where we don't have a previous target.
        else:
            best_value = float("inf") #if we have a previous target, we want to ignore confidence scores and only look at the euclidean distance to the last center. Therefore, the initial value is set to positive infinity.

        #loop over all results from the YOLO node
        for result in results:
            #loop over all bbox'es in the result, which contains the coordinates and confidence scores of the detected people.
            for box in result.boxes:
                confidence = float(box.conf[0]) #confidence score of the detection

                if confidence < self.confidence_threshold:
                    continue #ignore detections below the confidence threshold

                x1, y1, x2, y2 = box.xyxy[0].tolist() #bounding box coordinates converted to a list for easier handling

                #compute center of bounding box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Save center point and confidence score of this detection
                target = (x1, y1, x2, y2, center_x, center_y, confidence)

                #if we don't have a previous target we save the confidence score as the value to compare to other detections. 
                if self.last_center is None:
                    value = confidence

                    #if this detection is better than the best so far we save it as the best target to track.
                    if value > best_value:
                        best_value = value
                        best_target = target
                
                #if we have a previous target, we contunie to track them based on proximity to the last detected center point, ignoring confidence scores.
                else:
                    value = self.distance_from_last_center(center_x, center_y)
                    if value < best_value:
                        best_value = value
                        best_target = target
        return best_target  

    def distance_from_last_center(self, center_x, center_y):
        #This function computes the distance to the last published center in euclidian distance.
        
        #compute the difference in x and y coordinates between the current detection and the last published center
        dx = center_x - self.last_center[0]
        dy = center_y - self.last_center[1]

        #compute the Euclidean distance using the Pythagorean theorem and return (**0.5 is equivalent to the square root)
        return (dx * dx + dy * dy) ** 0.5
   
    def handle_lost_target(self, header):
        # Count each frame where no target is found.
        self.lost_frames += 1

        # if lost_frames exceeds the lost_frame_limit, reset the last center and publish an invalid target (valid=false).
        if self.lost_frames >= self.lost_frame_limit:
            self.last_center = None
            self.publish_invalid_target(header)

    def publish_invalid_target(self, header):
        #This function publishes an invalid target (valid=false)

        bbox = PersonBBox()
        bbox.header = header

        bbox.x1 = 0.0
        bbox.y1 = 0.0
        bbox.x2 = 0.0
        bbox.y2 = 0.0
        bbox.confidence = 0.0
        bbox.valid = False

        self.pub.publish(bbox)

    def publish_valid_target(self, target, header):
        #This function publishes the detected target as a PersonBBox message on the person_bbox topic and resets the lost frame count.

        #extract the coordinates, center point and confidence score from the target tuple for easier handling and readability.
        (x1, y1, x2, y2, center_x, center_y, confidence) = target

        #Update last center and reset lost frame count. Days since the last incident = zero ;D
        self.last_center = (center_x, center_y)
        self.lost_frames = 0

        bbox = PersonBBox()
        bbox.header = header
        bbox.x1 = float(x1)
        bbox.y1 = float(y1)
        bbox.x2 = float(x2)
        bbox.y2 = float(y2)
        bbox.confidence = float(confidence)
        bbox.valid = True

        self.pub.publish(bbox)

#typical ROS2 Python node main function, which initializes the node, spins it to keep it alive and handle shutdown gracefully when needed.
def main(args=None):
    rclpy.init(args=args)
    node = YoloPersonCenter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
