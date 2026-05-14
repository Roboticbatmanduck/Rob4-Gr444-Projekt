import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class DistanceRegulator (Node):

    def __init__(self):
        super().__init__('distance_regulator')

        # Declare parameters
        self.declare_parameter("reference", 1.85)
        self.declare_parameter("measured_topic", "/distance/measured")
        self.declare_parameter("output_topic", "/linear_velocity")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("publish_error", "/distance/error")
        self.declare_parameter("P_signal", "distance/P_signal")
        self.declare_parameter("I_signal", "distance/I_signal")
        self.declare_parameter("D_signal", "distance/D_signal")
        self.declare_parameter("min",0.0)
        self.declare_parameter("max",0.22)
        self.declare_parameter("kp",1.0)
        self.declare_parameter("ki",0.0)
        self.declare_parameter("kd",0.0)
        self.declare_parameter("deadband", 0.01)
        self.declare_parameter("timeout", 0.5)

        # Get parameters
        self.reference = float(self.get_parameter("reference").value)
        self.measured_topic = self.get_parameter("measured_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.error_topic = self.get_parameter("publish_error").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.P_topic = self.get_parameter("P_signal").value
        self.I_topic = self.get_parameter("I_signal").value
        self.D_topic = self.get_parameter("D_signal").value
        self.timeout = float(self.get_parameter("timeout").value)    
        self.min = float(self.get_parameter("min").value)
        self.max = float(self.get_parameter("max").value)
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.kd = float(self.get_parameter("kd").value)
        self.deadband = float(self.get_parameter("deadband").value)

        # Initialize control variables
        self.measured = None
        self.error = 0.0
        self.error_prev = 0.0
        self.u = 0.0
        self.I = 0.0
        self.last_msg = self.get_clock().now()

        #Subscriber for the measured distance
        self.create_subscription(
            Float32,
            self.measured_topic,
            self.measured_callback,
            10
        )

        # Create publishers 
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
        self.P_topic = self.create_publisher(
            Float32,
            self.P_topic,
            10
        )
        self.I_topic = self.create_publisher(
            Float32,
            self.I_topic,
            10
        )
        self.D_topic = self.create_publisher(
            Float32,
            self.D_topic,
            10
        )

        # Create control loop timer
        self.period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.period, self.compute_and_publish)

        #Log that the node has startet succesfully
        self.get_logger().info('Distance regulator started')

    def measured_callback(self, msg):
        # Store measured distance and timestamp
        self.measured = float(msg.data)
        self.last_msg = self.get_clock().now()
    
    def compute_and_publish(self):
        # Calculate error
        if self.measured is None:
            self.error = 0.0    
        else:
            self.error = self.measured - self.reference
        
        if abs(self.error) <= self.deadband:
            self.error = 0.0
        
        err = Float32()
        err.data = self.error
        self.error_publisher.publish(err)
        
        # Compute and publish control signal
        control_signal = self.compute_control()
        msg = Float32()
        msg.data = float(control_signal)
        self.control_publisher.publish(msg)

    def compute_control(self):
        # Check for timeout
        dt = self.get_clock().now() - self.last_msg
        seconds_since_last_msg = dt.nanoseconds * 1e-9
        if seconds_since_last_msg > self.timeout:
            return 0.0
        
        # Compute PID terms
        T = self.period
        P = self.kp * self.error
        I = self.ki * self.error * T + self.I
        I = np.clip(I, -self.max, self.max)
        D = self.kd * (self.error - self.error_prev) / T
        
        # Combine and clip output
        self.u = P + I + D
        self.u = np.clip(self.u, self.min, self.max)
        
        # Update state
        self.error_prev = self.error
        self.I = I
        
        # Publish PID components
        P_msg = Float32()
        I_msg = Float32()
        D_msg = Float32()
        P_msg.data = P
        I_msg.data = I
        D_msg.data = D
        self.P_topic.publish(P_msg)
        self.I_topic.publish(I_msg)
        self.D_topic.publish(D_msg)
        
        return self.u 
    
def main(args=None):
    rclpy.init(args=args)
    node = DistanceRegulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
