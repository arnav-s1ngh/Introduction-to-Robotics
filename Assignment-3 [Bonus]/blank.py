import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pi, cos, sin

class CircleTracer(Node):
    def __init__(self):
        super().__init__('circle_tracer')
        
        # Publisher for bot velocity
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Precompute the trajectory
        self.dt = 0.1  # Time step
        self.total_time = 15.0  # Total duration
        self.radius = 1.0
        self.linear_velocity = 2.0  # Constant linear velocity
        self.trajectory = []

        omega = self.linear_velocity / self.radius
        for t in [i * self.dt for i in range(int(self.total_time / self.dt) + 1)]:
            x = self.radius * cos(omega * t)
            y = self.radius * sin(omega * t)
            self.trajectory.append((x, y))

        # Internal state
        self.current_index = 0
        self.lookahead_distance = 0.5  # Lookahead distance for pure pursuit
        self.bot_position = [0.0, 0.0]  # Starting at origin
        self.bot_orientation = 0.0  # Orientation in radians

        # Timer for regular control updates
        self.timer = self.create_timer(self.dt, self.control_loop)

    def find_lookahead_point(self):
        bot_x, bot_y = self.bot_position
        for i in range(self.current_index, len(self.trajectory)):
            traj_x, traj_y = self.trajectory[i]
            distance = sqrt((traj_x - bot_x)**2 + (traj_y - bot_y)**2)
            if distance >= self.lookahead_distance:
                self.current_index = i
                return traj_x, traj_y
        return self.trajectory[-1]  # Fallback to the last point

    def control_loop(self):
        # Find the lookahead point
        lookahead_x, lookahead_y = self.find_lookahead_point()

        # Current bot position
        bot_x, bot_y = self.bot_position

        # Pure pursuit control
        error_x = lookahead_x - bot_x
        error_y = lookahead_y - bot_y
        lookahead_angle = atan2(error_y, error_x)

        # Normalize lookahead angle relative to bot orientation
        angle_error = lookahead_angle - self.bot_orientation
        while angle_error > pi:
            angle_error -= 2 * pi
        while angle_error < -pi:
            angle_error += 2 * pi

        # Control law
        angular_velocity = 2.0 * angle_error  # Proportional control for angular velocity

        # Set velocity command
        cmd = Twist()
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = angular_velocity

        # Publish the velocity command
        self.publisher.publish(cmd)

        # Update bot's position (basic simulation, replace with odom in real-world)
        self.bot_position[0] += self.linear_velocity * self.dt * cos(self.bot_orientation)
        self.bot_position[1] += self.linear_velocity * self.dt * sin(self.bot_orientation)
        self.bot_orientation += cmd.angular.z * self.dt

        self.get_logger().info(f"Lookahead: ({lookahead_x:.2f}, {lookahead_y:.2f}), Position: {self.bot_position}, Orientation: {self.bot_orientation:.2f}")

        # Stop after 15 seconds
        if self.current_index >= len(self.trajectory):
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
