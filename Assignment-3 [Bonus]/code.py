import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos, pi

class CircleTrackingController(Node):
    def __init__(self):
        super().__init__('circle_tracking_controller')

        # Publishers and Subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Parameters
        self.radius = 1.0  # Circle radius (meters)
        self.lookahead_distance = 0.008  # Lookahead distance
        self.linear_velocity = 2.0  # Linear velocity (m/s)
        self.pose = None
        self.timer_period = 0.01  # Time step (seconds)
        self.start_time = self.get_clock().now()

        # Timer
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        """Callback function to update the robot's current pose."""
        self.pose = msg.pose.pose

    def control_loop(self):
        if self.pose is None:
            return

        # Get current robot position and orientation
        x = self.pose.position.x
        y = self.pose.position.y
        qw = self.pose.orientation.w
        qz = self.pose.orientation.z
        theta = 2 * atan2(qz, qw)

        # Calculate current error and the lookahead point
        error = abs(sqrt(x ** 2 + y ** 2) - self.radius)
        current_angle = atan2(y, x)
        lookahead_angle = current_angle + self.linear_velocity * self.timer_period / self.radius
        x_ahead = self.radius * cos(lookahead_angle)
        y_ahead = self.radius * sin(lookahead_angle)

        # Calculate the control inputs
        dx = x_ahead - x
        dy = y_ahead - y
        alpha = atan2(dy, dx) - theta

        # Control law for angular velocity
        phi = atan(2 * (0.001 / self.lookahead_distance) * sin(alpha))
        angular_velocity = phi / self.timer_period

        # Publish control commands
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = angular_velocity
        self.publisher.publish(twist_msg)

        # Stop the robot after 15 seconds
        elapsed_time = self.get_clock().now() - self.start_time
        if elapsed_time.nanoseconds / 1e9 >= 15.0:
            self.timer.cancel()
            self.stop_robot()

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info("Stopping the robot.")


def main(args=None):
    rclpy.init(args=args)
    controller = CircleTrackingController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


