#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TurtlebotTrajectoryController(Node):
    def __init__(self):
        super().__init__('turtlebot_trajectory_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for odometry to track current position
        self.odom_subscription = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odometry_callback, 
            10
        )
        
        # Constant velocity
        self.velocity = 2.0
        
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # Path segments
        self.stages = [
            self.go_to_point,      # Stage 0: Go to (1,0)
            self.follow_circle     # Stage 1: Follow x^2 + y^2 = 1 anticlockwise
        ]
        self.current_stage = 0
    
    def quaternion_to_euler(self, quaternion):
        # Manual conversion of quaternion to euler angles
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
    
    def odometry_callback(self, msg):
        # Extract current position from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angles to get orientation
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.x, 
            orientation_q.y, 
            orientation_q.z, 
            orientation_q.w
        ]
        _, _, self.current_theta = self.quaternion_to_euler(quaternion)
    
    def go_to_point(self):
        # Go to point (1,0)
        dx = 1.0 - self.current_x
        dy = 0.0 - self.current_y
        
        # Calculate distance and angle to target
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # Create velocity command
        twist = Twist()
        
        # Rotate first if not aligned
        angle_diff = self.normalize_angle(target_angle - self.current_theta)
        if abs(angle_diff) > 0.1:
            twist.angular.z = 1.0 if angle_diff > 0 else -1.0
        else:
            # Move forward
            if distance > 0.05:
                twist.linear.x = self.velocity
            else:
                # Move to next stage
                self.current_stage += 1
        
        self.cmd_vel_pub.publish(twist)
        return distance > 0.05
    
    def follow_circle(self):
        # Follow x^2 + y^2 = 1 anticlockwise
        twist = Twist()
        
        # Calculate parametric equations for the circle
        t = math.atan2(self.current_y, self.current_x)
        
        # Desired next point on the circle
        next_x = math.cos(t + 0.1)
        next_y = math.sin(t + 0.1)
        
        # Calculate angle to next point
        target_angle = math.atan2(next_y, next_x)
        
        # Calculate angular and linear velocities
        angle_diff = self.normalize_angle(target_angle - self.current_theta)
        
        # Adjust heading and maintain velocity
        if abs(angle_diff) > 0.1:
            twist.angular.z = 1.0 if angle_diff > 0 else -1.0
        else:
            twist.linear.x = self.velocity
        
        self.cmd_vel_pub.publish(twist)
        
        # Check if we've completed a full circle
        return abs(t) < math.pi
    
    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def control_loop(self):
        # Execute current stage
        continue_stage = self.stages[self.current_stage]()
        
        # Stop if stage is complete
        if not continue_stage:
            # Stop the robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    controller = TurtlebotTrajectoryController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

