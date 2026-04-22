import rclpy
import os, json
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
import numpy as np


class StepInputNode(Node):
    def __init__(self):
        super().__init__("step_input")
        #Load parameters from json file 
        cfg ={}
        try:
            pkg_share = get_package_share_directory('wheelmodelling')
            json_path = os.path.join(pkg_share,"config", "step_input_params.json")
            with open(json_path, "r") as f:
                cfg = json.load(f)
        except Exception as e:
            self.get_logger().info(f'Could not locate parameters: {e}')
        


        #Declare parameters
        #the values for the parameters are read from the json file
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', cfg.get("publish_rate", 20.0)) #Hz
        self.declare_parameter('step_time', cfg.get("step_time", 1.0)) #s
        self.declare_parameter('step_amplitude', cfg.get("step_amplitude", 0.1)) #m/s
        self.declare_parameter('angular_z', cfg.get('angular_z', 0.0)) # rad/s
        self.declare_parameter('duration', cfg.get('duration', 5.0)) #s
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('imu_topic', '/imu')
        #Get parameters
        p = self.get_parameter
        self.cmd_topic = p('cmd_topic').value
        self.publish_rate = float(p('publish_rate').value)
        self.step_time = float(p('step_time').value)
        self.step_amplitude = float(p('step_amplitude').value)
        self.angular_z = float(p('angular_z').value)
        self.duration = float(p('duration').value)
        self.odom_topic = self.get_parameter('odom_topic').value
        self.joint_topic = self.get_parameter('joint_state_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        #Publisher and timer
        #send twist command to cmd_topic with 10 up to backlog
        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        #dt is the sampling time in seconds, 1/(20 Hz) = 0.05 second
        dt = 1.0/self.publish_rate if self.publish_rate > 0 else 0.05
        #creates the timer which runs every dt time, the function _on_timer
        self.timer = self.create_timer(dt, self._on_timer)

        #Quality of Service til sensor data (Best effort)
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST,
                                depth=10)
        
        #Subscriber til /odom topiccen
        self.subscriber_odom = self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos_sensor)
        self.subscriber_joint = self.create_subscription(JointState, self.joint_topic, self._on_joint_states, qos_sensor)
        self.subscriber_imu =  self.create_subscription(Imu, self.imu_topic, self._on_imu, qos_sensor)
        #Last log time
        self.last_log_t = 0.0
        #start time
        self.t0 = self.get_clock().now()
        #starts the logger with startup information
        self.get_logger().info(f'Publishing step on {self.cmd_topic} at {self.publish_rate}Hz '
                               f'step time = {self.step_time}s, step amplitude = {self.step_amplitude}m/s '
                               f'angular z = {self.angular_z}rad/s, duration = {self.duration}s')
        try:
            log_dir = os.path.expanduser('/workspace/logs')
            os.makedirs(log_dir, exist_ok=True)
            self.log_file = open(os.path.join(log_dir, 'speed_log.csv'), 'w')
            self.log_file.write("time,cmd_v,cmd_z,left_wheel,right_wheel,odom_yaw_rate,imu_yaw_rate\n")
            self.get_logger().warn(f'Logging to: {self.log_file.name}')
        except Exception as e:
            self.get_logger().warn(f'Error in logging: {e}')

        self.joint_left = 0.0
        self.joint_right = 0.0
        self.odom_z = 0.0
        self.yaw_rate = 0.0

    #Calculates the time from startup till now    
    def _now_sec(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    #Funcktionen der sørger for at lave et square-wave signal til motoren i en twist besked
    def _on_timer(self):
        t = self._now_sec()
        msg = Twist()
        phase = t % self.duration
        if phase >= self.step_time:
            msg.linear.x = self.step_amplitude
            msg.angular.z = self.angular_z
        else:
            msg.linear.x = self.step_amplitude
            msg.angular.z = 0.0
        cmd_v = msg.linear.x
        cmd_z = msg.angular.z
        self.publisher.publish(msg)
        self.log_file.write(f"{t},{cmd_v},{cmd_z},{self.joint_left},{self.joint_right},{self.odom_z},{self.yaw_rate}\n")
        self.log_file.flush()

    def _on_imu(self,msg):
        self.yaw_rate = msg.angular_velocity.z
        self.get_logger().info(f'yaw rate = {self.yaw_rate}')

    def _on_joint_states(self, msg):
        # JointState har: msg.name, msg.position, msg.velocity, msg.effort
        # Turtlebot3 har 2 hjul: left wheel, right wheel
        # velocity er i rad/s
        
        # Tag gennemsnittet for en samlet fremad-hastighed:
        if len(msg.velocity) >= 2:
            self.joint_left = msg.velocity[0]
            self.joint_right = msg.velocity[1]
            self.get_logger().info(f"left={self.joint_left:.5f}, right={self.joint_right:.5f}")
            speed = (self.joint_left + self.joint_right) / 2.0   # rad/s
            t = self._now_sec()
            self.joint_speed = speed

    def _on_odom(self, msg):
        v = msg.twist.twist.linear
        self.odom_z = msg.twist.twist.angular.z
        speed = np.sqrt(v.x**2 + v.y**2 + v.z**2)
        t = self._now_sec()
        self.get_logger().info(f't={t:5.2f}s, speed = {speed:5.3f}m/s, angular = {self.odom_z:5.3f}')
        # self.log_file.write(f"{t},{speed},{z}\n")
        self.log_file.flush()
        self.last_log_t = t
    
    def destroy_node(self):
        self.log_file.close()
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)
        return super().destroy_node()

def main():
    rclpy.init()
    node = StepInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    

if __name__ == "__main__":
    main()
