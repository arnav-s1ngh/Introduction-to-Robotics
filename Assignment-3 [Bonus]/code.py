import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class CircleController(Node):
    def __init__(self):
        super().__init__('circle_controller')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize robot's state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Parameters for the circular trajectory
        self.radius = 1.0
        self.linear_velocity = 2.0

        # Timer to update velocity commands
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        # Timer for 15-second duration
        self.start_time = self.get_clock().now()

    def odom_callback(self, msg):
        # Extract current position and orientation from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y**2 + orientation_q.z**2)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # Stop after 15 seconds
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed_time > 15.0:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)  # Stop the robot
            self.get_logger().info("15 seconds elapsed. Stopping the robot.")
            rclpy.shutdown()
            return

        # Calculate radial distance to origin
        radial_distance = math.sqrt(self.current_x**2 + self.current_y**2)

        # Calculate the current angle in polar coordinates
        current_angle = math.atan2(self.current_y, self.current_x)

        # Target angle for pure pursuit
        target_angle = current_angle + (self.linear_velocity / self.radius) * 0.01

        # Target position on the circle
        target_x = self.radius * math.cos(target_angle)
        target_y = self.radius * math.sin(target_angle)

        # Compute errors
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        # Control law for a unicycle model
        alpha = math.atan2(dy, dx) - self.current_theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # Normalize alpha to [-pi, pi]
        
        angular_velocity = 2 * math.sin(alpha)

        # Publish velocity commands
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
