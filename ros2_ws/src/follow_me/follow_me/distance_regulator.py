import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class DistanceRegulator (Node):
    """ROS2 node that regulates the robot's linear motion. 
    The node recieves a distance reference and a measured distance,
    computes the error, and output a linear velocity command."""

    def __init__(self):
        super().__init__('distance_regulator')

        self.declare_parameter("reference", 1.85)
        self.declare_parameter("measured_topic", "/distance/measured")
        self.declare_parameter("output_topic", "/linear_velocity")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("publish_error", "/distance/error")
        self.declare_parameter("min",0.0)
        self.declare_parameter("max",0.22)
        self.declare_parameter("kp",1.0)
        self.declare_parameter("ki",0.0)
        self.declare_parameter("kd",0.0)
        self.declare_parameter("deadband", 0.01)

        # Get parameters
        self.reference = float(self.get_parameter("reference").value)
        self.measured_topic = self.get_parameter("measured_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.error_topic = self.get_parameter("publish_error").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
    
        self.min = float(self.get_parameter("min").value)
        self.max = float(self.get_parameter("max").value)
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.deadband = float(self.get_parameter("deadband").value)

        self.measured = self.reference #Initialize the measured distance to the reference to avoid large initial error
        self.error = 0.0
        self.error_prev = 0.0
        self.error_old = 0.0
        self.u = 0.0
        self.u_prev = 0.0
        self.I = 0.0
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
        self.error_publisher = self.create_publisher(
            Float32,
            self.error_topic,
            10
        )

        #Timer that runs the control loop at 20 Hz
        self.period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.period, self.compute_and_publish)

        #Log that the node has startet succesfully
        self.get_logger().info('Distance regulator started')

    def measured_callback(self, msg):
        #Callback function for the measured distance. Stores the latest value
        self.measured = float(msg.data)
    
    def compute_and_publish(self):
        """Computes the control error and publishes the control signal."""
        #Calculate the control error
        err = Float32()
        self.error = self.measured - self.reference
        err.data = self.error
        self.error_publisher.publish(err)
        #Compute control signal using the regulator
        control_signal = self.compute_control()

        #Publiosh the control singal as linear velocity
        msg = Float32()
        msg.data = float(control_signal)
        
        self.control_publisher.publish(msg)

    def compute_control(self):
        #Regulatoren altså PID/Lead lag led indsættes her
        T = self.period
        P = self.kp * self.error
        I = self.ki * self.error*T + self.I
        D = self.kd*(self.error - self.error_prev)/T
        self.u = P + I + D
        self.u = np.clip(self.u, self.min, self.max) #Clip the control signal to be between self.min and self.max
        self.error_old = self.error_prev
        self.error_prev = self.error
        self.I = I
        self.u_prev = self.u
        if abs(self.u) < self.deadband:
            return 0.0
        return self.u 
    
def main(args=None):
    """Main function that initializes ROS2 and starts the node"""
    rclpy.init(args=args)
    node = DistanceRegulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()