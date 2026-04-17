import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from turtlebot3_msgs.msg import SensorState

# Helper function to convert quaternion to yaw angle
def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AngleModelling(Node):
    def __init__(self):
        super().__init__('angle_modelling')

        # Declare parameters with default values
        self.declare_parameter('forward_velocity', 0.0) # m/s
        self.declare_parameter('angular_velocity', 0.0) # rad/s
        self.declare_parameter('publish_rate', 20.0) # Hz
        self.declare_parameter('log_file_path', 'angle_modelling_data.csv')
        self.declare_parameter('duration', 0.0) # seconds
        self.declare_parameter('stop_log_duration', 1.0) # seconds to continue logging after stopping the robot

        # Get parameter values
        self.forward_velocity = float(self.get_parameter('forward_velocity').value)
        self.angular_velocity = float(self.get_parameter('angular_velocity').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.log_file_path = self.get_parameter('log_file_path').value
        self.duration = float(self.get_parameter('duration').value)
        self.stop_log_duration = float(self.get_parameter('stop_log_duration').value)

        # Initialize timer and start time
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # State IMU and SensorState data availability
        self.have_imu = False
        self.have_sensor_state = False

        # Initialize variables to store current IMU data
        self.current_yaw = 0.0
        self.current_imu_ang_vel_z = 0.0
        self.current_imu_stamp = 0.0

        # Initialize variables to store current SensorState data
        self.left_encoder = 0
        self.right_encoder = 0
        self.current_sensor_stamp = 0.0

        # State to track if the robot has been stopped after duration is reached
        self.stopping = False
        self.stop_start_time = None

        # Create publisher and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu',self.imu_callback, 50)
        self.sensor_state_sub = self.create_subscription(SensorState, '/sensor_state', self.sensor_state_callback, 50)

        # Create timer for publishing commands and logging data
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        #csv file setup
        self.csv_path = Path(self.log_file_path)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'node_time_sec',
            'elapsed_sec',
            'cmd_forward_speed',
            'cmd_angular_speed',
            'imu_time_sec',
            'imu_ang_z',
            'imu_ang_vel_z',
            'sensor_state_time',
            'left_encoder',
            'right_encoder'
        ])

        # Write initial log message with parameter values
        self.get_logger().info('Angle Modelling Node Initialized')
        self.get_logger().info(
            f"forward_velocity={self.forward_velocity}, "
            f"angular_velocity={self.angular_velocity}, "
            f"publish_rate={self.publish_rate}, "
            f"log_file_path={self.log_file_path}, "
            f"duration={self.duration}, "
            f"stop_log_duration={self.stop_log_duration}"
        )

    # Function for IMU callback to update current yaw and angular velocity
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_imu_ang_vel_z = msg.angular_velocity.z
        self.current_imu_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.have_imu = True

    # Function for SensorState callback to update current encoder values
    def sensor_state_callback(self, msg: SensorState):
        self.left_encoder = msg.left_encoder
        self.right_encoder = msg.right_encoder
        self.current_sensor_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.have_sensor_state = True
    
    def publish_cmd(self, linear_x: float, angular_z: float):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def log_row(self, now_time: float, elapsed_time: float, cmd_linear_x: float, cmd_angular_z: float):
        self.csv_writer.writerow([
            f'{now_time:.6f}',
            f'{elapsed_time:.6f}',
            f'{cmd_linear_x:.6f}',
            f'{cmd_angular_z:.6f}',
            f'{self.current_imu_stamp:.6f}' if self.have_imu else '',
            f'{self.current_yaw:.6f}' if self.have_imu else '',
            f'{self.current_imu_ang_vel_z:.6f}' if self.have_imu else '',
            f'{self.current_sensor_stamp:.6f}' if self.have_sensor_state else '',
            self.left_encoder if self.have_sensor_state else '',
            self.right_encoder if self.have_sensor_state else ''
        ])
        self.csv_file.flush()

    # Timer callback to publish velocity commands and log data to CSV file
    def timer_callback(self):
        now_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = now_time - self.start_time

        if not self.stopping and self.duration > 0 and elapsed_time >= self.duration:
            self.get_logger().info('Duration reached, entering stopping phase.')
            self.stopping = True
            self.stop_start_time = now_time

        if self.stopping:
            stop_elapsed = now_time - self.stop_start_time
            self.publish_cmd(0.0, 0.0)
            self.log_row(now_time, elapsed_time, 0.0, 0.0)

            if stop_elapsed >= self.stop_log_duration:
                self.get_logger().info('Stop log duration reached, shutting down.')
                self.csv_file.flush()
                self.csv_file.close()
                rclpy.shutdown()
            return

        self.publish_cmd(self.forward_velocity, self.angular_velocity)
        self.log_row(
            now_time,
            elapsed_time,
            self.forward_velocity,
            self.angular_velocity
        )

    # Function to stop the robot by publishing zero velocities
    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)
        self.get_logger().info('Robot stopped.')
    
    def destroy_node(self):
        try:
            self.stop_robot()
        except Exception:
            pass

        try:
            if not self.csv_file.closed:
                self.csv_file.close()
        except Exception:
            pass

        return super().destroy_node()

# Main function to initialize the node and handle shutdown
def main(args=None):
    rclpy.init(args=args)
    node = AngleModelling()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down.')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()

"""
Example of run command with all parameters
ros2 run anglemodelling anglemodelling_node \
  --ros-args \
  -p forward_velocity:=0.0 \
  -p angular_velocity:=0.0 \
  -p publish_rate:=20.0 \
  -p duration:=5.0 \
  -p stop_log_duration:=2.0 \
  -p log_file_path:=/home/oliver/angle_log.csv
"""