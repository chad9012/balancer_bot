import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from time import time

# PID Controller class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Robot Balance Node
class BalanceRobotNode(Node):
    def __init__(self):
        super().__init__('balance_robot_node')

        # PID coefficients
        self.kp = 0.3  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.1  # Derivative gain

        # PID controller instance
        self.pid = PIDController(self.kp, self.ki, self.kd)

        # Create a publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create a publisher for PID error
        self.pid_error_pub = self.create_publisher(Float64, '/pid_error', 10)

        # Create a subscriber to IMU data
        self.imu_sub = self.create_subscription(Imu, '/imu_sensor_plugin/out', self.imu_callback, 10)

        # Initialize variables
        self.prev_time = time()
        self.prev_angle = 0.0
        self.filtered_cmd_vel = 0.0  # Variable to store the filtered velocity command
        self.alpha = 0.1  # Smoothing factor for low-pass filter

    def imu_callback(self, msg):
        # Extract pitch angle from IMU data (you can adjust based on your robot's orientation)
        pitch = msg.linear_acceleration.x  # assuming pitch is represented by x-axis of linear acceleration

        # Compute the error (desired angle is 0, meaning balanced)
        error = pitch  # For simplicity, we use pitch as the angle to correct

        # Get current time and compute time difference for PID calculation
        current_time = time()
        delta_time = current_time - self.prev_time
        self.prev_time = current_time

        # Compute PID output for controlling velocity
        pid_output = self.pid.compute(error)

        # Apply the low-pass filter to the PID output
        self.filtered_cmd_vel = self.alpha * pid_output + (1 - self.alpha) * self.filtered_cmd_vel

        # Create and send the command to control the robot's wheel velocity
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -self.filtered_cmd_vel  # Set forward speed (can be adjusted based on your needs)
        cmd_vel_msg.angular.z = 0.0  # Adjust rotational velocity based on PID output

        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd_vel_msg)

        # Log the angle (pitch) and PID command
        self.get_logger().info(f"Pitch Angle: {pitch}, PID Command (linear.x): {cmd_vel_msg.linear.x}")

        # Publish PID error for monitoring
        pid_error_msg = Float64()
        pid_error_msg.data = error
        self.pid_error_pub.publish(pid_error_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = BalanceRobotNode()

    # Spin the node to keep it running
    rclpy.spin(node)

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
