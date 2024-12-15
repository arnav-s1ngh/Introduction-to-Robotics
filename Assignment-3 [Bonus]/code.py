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
        self.linear_velocity = 2.0  # Linear velocity (m/s)
        self.pose = None
        self.timer_period = 0.01  # Time step (seconds)
        self.start_time = self.get_clock().now()

        # Timer
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        """Callback function to update the robot's current pose."""
        self.pose = msg.pose.pose

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = sin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)

        return roll, pitch, yaw

    def control_loop(self):
        if self.pose is None:
            return

        # Get current robot position and orientation
        x = self.pose.position.x
        y = self.pose.position.y
        orientation_q = self.pose.orientation
        _, _, theta = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        # Calculate the target point on the circle
        target_x = self.radius * cos(self.linear_velocity * self.timer_period / self.radius)
        target_y = self.radius * sin(self.linear_velocity * self.timer_period / self.radius)

        # Calculate error and desired angle
        dx = target_x - x
        dy = target_y - y
        distance_error = sqrt(dx**2 + dy**2)
        desired_angle = atan2(dy, dx)
        angle_error = desired_angle - theta

        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + pi) % (2 * pi) - pi

        # Publish control commands
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        twist_msg.angular.z = 2.0 * angle_error  # Proportional controller for angular velocity
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


