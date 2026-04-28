import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np


class command_sender(Node):
    def __init__(self):
        super().__init__('Command_Sender') # Node name
        self.get_logger().info('Command_Sender Startet') 
        self.linear = 0.0 # Initiate variables
        self.angular = 0.0
        self.last_msg = self.get_clock().now()

        self.declare_parameter('publish_rate',20.0)
        self.declare_parameter('timeout',0.5)
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('max_velocity', 2.84)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_velocity').value

        self.create_subscription(Float32,               # Create subscriber to linear velocity topic, with default qos
                                '/linear_velocity',
                                self.linear_callback,
                                10) 
        self.create_subscription(Float32,               # Create subscriber to angular velocity topic with default qos
                                '/angular_velocity',
                                self.angular_callback,
                                10)
        
        # Default qos is:
        # Keep last: only store up to 10 samples
        # Reliable: guarantee that samples are delivered, may retry multiple times
        # Volatile: no attempt is made to persist samples
        # liveliness: system default
        # Deadline: default 
        # lifespan: default
        # lease durations: default
        # On https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html, it does not specify what default is

        self.publisher = self.create_publisher(Twist,       #Create publisher to /cmd_vel with Twist message
                            '/cmd_vel',
                            10)
        period = 1/self.publish_rate # publish with 20 Hz
        self.timer = self.create_timer(period,self.timer_callback) #Create the timer with the specified period

    def linear_callback(self,msg):  #Linear velocity callback, which logs the recieved data and clips it to max values
        self.get_logger().info(f'Recieved: {msg.data} from /linear_velocity')
        self.linear = msg.data
        self.linear = np.clip(self.linear,0,self.max_linear)
        self.last_msg = self.get_clock().now()


    def angular_callback(self,msg): #Angular velocity callback, which logs the recieved data and clips it to max values
        self.get_logger().info(f'Recieved: {msg.data} from /angular_velocity')
        self.angular = msg.data
        self.angular = np.clip(self.angular, -self.max_angular, self.max_angular)
        self.last_msg = self.get_clock().now()


    def timer_callback(self): #Timer callback, this happends every time the defined period is up, it publshes the recieved linear and angular velocities to the /cmd_vel topic
        dt = self.get_clock().now() - self.last_msg
        msg = Twist()
        if dt.nanoseconds*1e-9 > self.timeout:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            msg.linear.x = self.linear
            msg.angular.z = self.angular
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: liniear.x = {self.linear}[m/s], and angular.z={self.angular}[rad/s] to /cmd_vel')


def main(args=None): #main function running all the code
    rclpy.init(args=args) #initialises the ros2 system for python, it starts the DDS (args=args) allows it to read ros arguments from the CLI
    node = command_sender() #creates a node object with the subsribers, publishers and timers
    rclpy.spin(node) #Starts the node, begins subsribing and publishing to topics, and keeps it here until it is stopped
    node.destroy_node() #Stops the node neatly
    rclpy.shutdown() #closes the connection to the ros2 system

if __name__ == "__main__":
    main()