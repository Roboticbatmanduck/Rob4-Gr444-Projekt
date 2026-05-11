import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class AngleRegulator (Node):
    """
    Node that regulates the robot's angular motion. 
    Recieves a reference and measured angle, computes the error,
    and output an angular velocity command
    """

    def __init__(self):
        super().__init__('angle_regulator')

        self.declare_parameter("reference", 15.0)
        self.declare_parameter("measured_topic", "/angle/measured")
        self.declare_parameter("output_topic", "/angular_velocity")
        self.declare_parameter("publish_rate", 20.0)

        # Get parameters
        self.reference = float(self.get_parameter("reference").value)
        self.measured_topic = self.get_parameter("measured_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.measured = self.reference # Initialize measured angle to reference to avoid large initial error

        #Subscriber for the measured angle
        self.create_subscription(
            Float32,
            self.measured_topic,
            self.measured_callback,
            10
        )

        #Publisher for the angular velocity command
        self.control_publisher = self.create_publisher(
            Float32,
            self.output_topic,
            10
        )

        #Timer that runs the control loop at 20 Hz
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.compute_and_publish)

        #Log that the node has startet succesfully
        self.get_logger().info('Angle regulator started')

    def measured_callback(self, msg):
        #Callback function for the measured angle. Stores the latest value
        self.measured = float(msg.data)
    
    def compute_and_publish(self):
        """Computes the control error and publishes the control signal.
        """

        #Calculate the error
        error = self.reference - self.measured

        #Compute control signal using the regulator
        control_signal = self.compute_control(error)

        #Publish the control signal as angular velocity
        msg = Float32()
        msg.data = float(control_signal)
        self.control_publisher.publish(msg)

    def compute_control(self, error):
        #Regulatoren altså PID/Lead lag led indsættes her

        return error #midlertidig "P" regulator
    
def main(args=None):
    """Main function that initializes ROS2 and starts the node"""
    rclpy.init(args=args)
    node = AngleRegulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()