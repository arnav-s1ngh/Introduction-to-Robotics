#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Parameters
        self.radius = 1.0  # Circle radius
        self.linear_velocity = 2.0  # Constant linear velocity
        self.look_ahead_distance = 1.6  # Look-ahead distance
        
        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.current_pose = None
        self.start_time = self.get_clock().now()

    def quaternion_to_euler(self, quaternion):
        """
        Convert quaternion to Euler angles (yaw)
        Manual implementation without tf_transformations
        """
        x, y, z, w = quaternion
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, yaw = self.quaternion_to_euler([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_pose = (pose.position.x, pose.position.y, yaw)

    def calculate_control_command(self, current_pose):
        """Calculate control commands for pure pursuit tracking"""
        x, y, theta = current_pose

        # Calculate current angle on the circle
        current_angle = math.atan2(y, x)
        
        # Calculate look-ahead point on the circle
        look_ahead_angle = current_angle + (self.linear_velocity * 0.1) / self.radius
        x_ahead = self.radius * math.cos(look_ahead_angle)
        y_ahead = self.radius * math.sin(look_ahead_angle)

        # Calculate path tracking errors
        dx = x_ahead - x
        dy = y_ahead - y

        # Calculate steering angle
        alpha = math.atan2(dy, dx) - theta
        
        # Calculate angular velocity
        omega = 2.0 * math.sin(alpha) / self.look_ahead_distance

        return self.linear_velocity, omega

    def control_loop(self):
        """Main control loop for tracking circle path"""
        # Check if pose is available
        if not self.current_pose:
            return

        # Check elapsed time
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        
        # Stop after 15 seconds
        if elapsed_time > 15.0:
            self.stop_robot()
            return

        # Calculate and publish velocities
        linear_vel, angular_vel = self.calculate_control_command(self.current_pose)
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stop the robot when tracking is complete"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.destroy_timer(self.control_timer)
        self.get_logger().info('Pure Pursuit tracking completed.')

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

