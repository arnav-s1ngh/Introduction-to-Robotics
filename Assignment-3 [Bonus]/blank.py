import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pi

class CircleTracer(Node):
    def __init__(self):
        super().__init__('circle_tracer')
        
        # Publisher for bot velocity
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to update velocity at regular intervals
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Desired trajectory
        self.radius = 1.0
        self.linear_velocity = 2.0  # Constant linear velocity
        
        # Internal state
        self.time_elapsed = 0.0
        self.dt = 0.1  # Time step
        self.bot_position = [0.0, 0.0]  # Starting at origin
        self.bot_orientation = 0.0  # Orientation in radians

    def control_loop(self):
        # Calculate the desired position on the circle at current time
        self.time_elapsed += self.dt
        desired_x = self.radius * cos(self.linear_velocity * self.time_elapsed / self.radius)
        desired_y = self.radius * sin(self.linear_velocity * self.time_elapsed / self.radius)

        # Current bot position
        bot_x, bot_y = self.bot_position

        # Compute errors
        error_x = desired_x - bot_x
        error_y = desired_y - bot_y
        
        # Proportional control for angular velocity
        desired_angle = atan2(error_y, error_x)
        angle_error = desired_angle - self.bot_orientation

        # Normalize angle error to [-pi, pi]
        while angle_error > pi:
            angle_error -= 2 * pi
        while angle_error < -pi:
            angle_error += 2 * pi

        # Set velocity command
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = 2.0 * angle_error  # Gain for angular velocity

        # Publish the velocity command
        self.publisher.publish(cmd)

        # Update bot's position (basic simulation, replace with odom in real-world)
        self.bot_position[0] += self.linear_velocity * self.dt * cos(self.bot_orientation)
        self.bot_position[1] += self.linear_velocity * self.dt * sin(self.bot_orientation)
        self.bot_orientation += cmd.angular.z * self.dt

        self.get_logger().info(f"Time: {self.time_elapsed:.2f}s, Position: {self.bot_position}, Orientation: {self.bot_orientation:.2f}")

        # Stop after 15 seconds
        if self.time_elapsed >= 15.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd)
            self.get_logger().info("Finished tracing the circle.")
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CircleTracer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
