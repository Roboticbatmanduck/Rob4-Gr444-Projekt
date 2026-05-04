import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DistanceRegulator (Node):
    """ROS2 node that regulates the robot's linear motion. 
    The node recieves a distance reference and a measured distance,
    computes the error, and output a linear velocity command."""

    def __init__(self):
        super().__init__('distance_regulator')

        self.declare_parameter("reference", 2.0)
        self.declare_parameter("measured_topic", "/distance/measured")
        self.declare_parameter("output_topic", "/linear_velocity")
        self.declare_parameter("publish_rate", 20.0)

        # Get parameters
        self.reference = float(self.get_parameter("reference").value)
        self.measured_topic = self.get_parameter("measured_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.measured = self.reference #Initialize the measured distance to the reference to avoid large initial error

        #Subscriber for the measured distance
        self.create_subscription(
            Float32,
            self.measured_topic,
            self.measured_callback,
            10
        )

        #Publisher for the linear velocity command
        self.control_publisher = self.create_publisher(
            Float32,
            self.output_topic,
            10
        )

        #Timer that runs the control loop at 20 Hz
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.compute_and_publish)

        #Log that the node has startet succesfully
        self.get_logger().info('Distance regulator started')

    def measured_callback(self, msg):
        #Callback function for the measured distance. Stores the latest value
        self.measured = float(msg.data)
    
    def compute_and_publish(self):
        """Computes the control error and publishes the control signal."""
        #Calculate the control error
        error = self.measured - self.reference

        #Compute control signal using the regulator
        control_signal = self.compute_control(error)

        #Publiosh the control singal as linear velocity
        msg = Float32()
        msg.data = float(control_signal)
        
        self.control_publisher.publish(msg)

    def compute_control(self, error):
        #Regulatoren altså PID/Lead lag led indsættes her

        return error #midlertidig "P" regulator
    
def main(args=None):
    """Main function that initializes ROS2 and starts the node"""
    rclpy.init(args=args)
    node = DistanceRegulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()