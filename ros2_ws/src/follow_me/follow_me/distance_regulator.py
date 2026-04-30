import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DistanceRegulator (Node):
    """ROS2 node that regulates the robot's linear motion. 
    The node recieves a distance reference and a measured distance,
    computes the error, and output a linear velocity command."""

    def __init__(self):
        super().__init__('distance_regulator')

        #Store the leatest reference and measured distance values
        self.reference = 0.0
        self.measured = 0.0

        #Subscriber for the reference distance
        self.create_subscription(
            Float32,
            '/distance/reference',
            self.reference_callback,
            10
        )

        #Subscriber for the measured distance
        self.create_subscription(
            Float32,
            '/distance/measured',
            self.measured_callback,
            10
        )

        #Publisher for the linear velocity command
        self.control_publisher = self.create_publisher(
            Float32,
            '/linear_velocity',
            10
        )

        #Timer that runs the control loop at 20 Hz
        period = 1.0 / 20.0
        self.timer = self.create_timer(period, self.compute_and_publish)

        #Log that the node has startet succesfully
        self.get_logger().info('Distance regulator started')


    def reference_callback(self, msg):
        #Callback function for the distance reference. Stores the latest value
        self.reference = msg.data

    def measured_callback(self, msg):
        #Callback function for the measured distance. Stores the latest value
        self.measured = msg.data
    
    def compute_and_publish(self):
        """Computes the control error and publishes the control signal."""
        #Calculate the control error
        error = self.reference - self.measured

        #Compute control signal using the regulator
        control_signal = self.compute_control(error)

        #Publiosh the control singal as linear velocity
        msg = Float32()
        msg.data = control_signal
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