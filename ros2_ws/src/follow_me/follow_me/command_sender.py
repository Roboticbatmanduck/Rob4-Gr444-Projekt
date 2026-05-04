import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np


class CommandSender(Node):
    def __init__(self):
        super().__init__('command_sender') # Node name
         
        self.declare_parameter("linear_topic", "/linear_velocity")
        self.declare_parameter("angular_topic", "/angular_velocity")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("max_linear", 0.22)
        self.declare_parameter("max_angular", 2.84)

        # Get parameters
        self.linear_topic = self.get_parameter("linear_topic").value
        self.angular_topic = self.get_parameter("angular_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.linear = 0.0
        self.angular = 0.0
        self.last_msg = self.get_clock().now()

        self.create_subscription(# Create subscriber to linear velocity topic, with default qos
            Float32,               
            self.linear_topic,
            self.linear_callback,
            10
            )
         
        self.create_subscription(# Create subscriber to angular velocity topic with default qos
            Float32,               
            self.angular_topic,
            self.angular_callback,
            10
            )
        
        # Default qos is:
        # Keep last: only store up to 10 samples
        # Reliable: guarantee that samples are delivered, may retry multiple times
        # Volatile: no attempt is made to persist samples
        # liveliness: system default
        # Deadline: default 
        # lifespan: default
        # lease durations: default
        # On https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html, it does not specify what default is

        self.publisher = self.create_publisher(#Create publisher to /cmd_vel with Twist message
            Twist,
            self.cmd_vel_topic,
            10
            )
        
        period = 1.0 / self.publish_rate # publish with 20 Hz
        self.timer = self.create_timer(period, self.timer_callback) #Create the timer with the specified period

        self.get_logger().info('Command_Sender Startet')

    def linear_callback(self,msg):  #Linear velocity callback, which logs the recieved data and clips it to max values
        self.linear = float(msg.data)
        self.linear = float(np.clip(self.linear,0.0,self.max_linear))
        self.last_msg = self.get_clock().now()
        self.get_logger().debug(f'Recieved: {msg.data} from {self.linear_topic}, clipped to {self.linear}')


    def angular_callback(self,msg): #Angular velocity callback, which logs the recieved data and clips it to max values
        self.angular = float(msg.data)
        self.angular = float(np.clip(self.angular, -self.max_angular, self.max_angular))
        self.last_msg = self.get_clock().now()
        self.get_logger().debug(f'Recieved: {msg.data} from {self.angular_topic}, clipped to {self.angular}')


    def timer_callback(self): #Timer callback, this happends every time the defined period is up, it publshes the recieved linear and angular velocities to the /cmd_vel topic
        dt = self.get_clock().now() - self.last_msg
        seconds_since_last_msg = dt.nanoseconds * 1e-9
        self.get_logger().debug(f'Time since last message: {seconds_since_last_msg:.3f} seconds')

        msg = Twist()

        if seconds_since_last_msg > self.timeout:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        else:
            msg.linear.x = self.linear
            msg.angular.z = self.angular
        
        self.publisher.publish(msg)
        
        self.get_logger().debug(f'Published: linear.x = {msg.linear.x}[m/s], and angular.z={msg.angular.z}[rad/s] to {self.cmd_vel_topic}')


def main(args=None): #main function running all the code
    rclpy.init(args=args) #initialises the ros2 system for python, it starts the DDS (args=args) allows it to read ros arguments from the CLI
    node = CommandSender() #creates a node object with the subsribers, publishers and timers
    rclpy.spin(node) #Starts the node, begins subsribing and publishing to topics, and keeps it here until it is stopped
    node.destroy_node() #Stops the node neatly
    rclpy.shutdown() #closes the connection to the ros2 system

if __name__ == "__main__":
    main()