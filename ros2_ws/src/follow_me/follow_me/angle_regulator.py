import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

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
        self.declare_parameter("error_topic", "angular/error")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("kp",1.0)
        self.declare_parameter("ki",0.0)
        self.declare_parameter("kd",0.0)
        self.declare_parameter("min",-2.84)
        self.declare_parameter("max",2.84)

        # Get parameters
        self.reference = float(self.get_parameter("reference").value)
        self.measured_topic = self.get_parameter("measured_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.error_topic = self.get_parameter("error_topic").value
        self.min = float(self.get_parameter("min").value)
        self.max = float(self.get_parameter("max").value)
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)

        self.measured = self.reference # Initialize measured angle to reference to avoid large initial error
        self.error = 0.0
        self.error_prev = 0.0
        self.error_old = 0.0
        self.u = 0.0
        self.u_prev = 0.0

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
        self.error_publisher = self.create_publisher(
            Float32,
            self.error_topic,
            10
        )

        #Timer that runs the control loop at 20 Hz
        self.period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.period, self.compute_and_publish)

        #Log that the node has startet succesfully
        self.get_logger().info('Angle regulator started')

    def measured_callback(self, msg):
        #Callback function for the measured angle. Stores the latest value
        self.measured = float(msg.data)
    
    def compute_and_publish(self):
        """Computes the control error and publishes the control signal.
        """

        #Calculate the error
        err = Float32()
        self.error = self.reference - self.measured
        err.data = self.error
        self.error_publisher.publish(err)
        #Compute control signal using the regulator
        control_signal = self.compute_control()

        #Publish the control signal as angular velocity
        msg = Float32()
        msg.data = float(control_signal)

        self.control_publisher.publish(msg)

    def compute_control(self):
        #Regulatoren altså PID/Lead lag led indsættes her

        T = self.period
        P = self.kp*(self.error-self.error_prev)
        I = self.ki*self.error*T
        D = self.kd*(self.error-2*self.error_prev+self.error_old)/T
        self.u = self.u_prev + P+I+D
        self.u = np.clip(self.u, self.min, self.max) #Clip the control signal to be between self.min and self.max
        self.error_old = self.error_prev
        self.error_prev = self.error
        self.u_prev = self.u
        return self.u 
    
def main(args=None):
    """Main function that initializes ROS2 and starts the node"""
    rclpy.init(args=args)
    node = AngleRegulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()