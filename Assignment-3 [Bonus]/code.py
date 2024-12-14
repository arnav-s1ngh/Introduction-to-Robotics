#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
import math
import numpy as np


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_publisher_ = self.create_publisher(Path, '/robot_path', 10)

        # Parameters
        self.radius = 1.0
        self.v = 2.0  # linear velocity
        self.ld = 0.008  # look-ahead distance
        self.dt = 0.005  # time step
        self.total_time = 15.0

        # Initial pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # For tracking
        self.xpath = []
        self.ypath = []
        self.thetapath = []
        self.errorpath = []

        # Timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Stop flag
        self.stop_flag = False

    def control_loop(self):
        # Store current state
        self.xpath.append(self.x)
        self.ypath.append(self.y)
        self.thetapath.append(self.theta % (2 * math.pi))

        # Calculate radial error
        current_radius = math.sqrt(self.x ** 2 + self.y ** 2)
        error = abs(current_radius - self.radius)
        self.errorpath.append(error)

        # Publish current path
        self.publish_path()

        # Calculate look-ahead point
        current_angle = math.atan2(self.y, self.x)
        ld_angle = current_angle + self.v * self.dt / self.radius

        x_ahead = self.radius * math.cos(ld_angle)
        y_ahead = self.radius * math.sin(ld_angle)

        # Calculate control inputs
        dx = x_ahead - self.x
        dy = y_ahead - self.y

        # Update position
        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt

        # Calculate steering angle
        alpha = math.atan2(dy, dx) - self.theta

        # Unicycle model steering
        wheel_radius = 0.001
        phi = math.atan(2 * (wheel_radius / self.ld) * math.sin(alpha))

        # Update heading
        self.theta += phi

        # Publish velocity command
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = phi
        self.publisher_.publish(twist)

        # Stop condition
        if len(self.xpath) >= self.total_time / self.dt:
            self.stop_flag = True
            self.stop_robot()

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in zip(self.xpath, self.ypath):
            pose = Point()
            pose.x = x
            pose.y = y
            path_msg.poses.append(pose)

        self.path_publisher_.publish(path_msg)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_controller=PurePursuitController()
    try:
        while rclpy.ok() and not pure_pursuit_controller.stop_flag:
            rclpy.spin_once(pure_pursuit_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pure_pursuit_controller.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
