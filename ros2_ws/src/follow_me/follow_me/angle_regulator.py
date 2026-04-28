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

        #Store the latest reference and measured angle values
        self.reference = 0.0
        self.measured = 0.0

        #Subscriber for the reference angle
        self.create_subscription(
            Float32,
            '/angle/reference',
            self.reference_callback,
            10
        )

        #Subscriber for the measured angle
        self.create_subscription(
            Float32,
            '/angle/measured',
            self.measured_callback,
            10
        )

        #Publisher for the angular velocity command
        self.control_publisher = self.create_publisher(
            Float32,
            '/angular_velocity',
            10
        )

        #Timer that runs the control loop at 20 Hz
        period = 1.0 / 20.0
        self.timer = self.create_timer(period, self.compute_and_publish)

        #Log that the node has startet succesfully
        self.get_logger().info('Angle regulator started')


    def reference_callback(self, msg):
        #Callback function for the reference angle. Store the latest value
        self.reference = msg.data

    def measured_callback(self, msg):
        #Callback function for the measured angle. Stores the latest value
        self.measured = msg.data
    
    def compute_and_publish(self):
        """Computes the control error and publishes the control signal.
        """

        #Calculate the error
        error = self.reference - self.measured

        #Compute control signal using the regulator
        control_signal = self.compute_control(error)

        #Publish the control signal as angular velocity
        msg = Float32()
        msg.data = control_signal
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