import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, Pose
import math
import time

class CircleController(Node):
    def __init__(self):
        super().__init__('circle_controller')
        self.desired_velocity = 2.0
        self.circle_radius = 1.0
        self.current_pose = Pose()
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def pose_callback(self, msg):
        self.current_pose = msg

    def timer_callback(self):
        error_x = self.circle_radius * math.cos(self.desired_velocity * time.time()) - self.current_pose.position.x
        error_y = self.circle_radius * math.sin(self.desired_velocity * time.time()) - self.current_pose.position.y
        control_output_x = error_x * 2.0
        control_output_y = error_y * 2.0
        msg = Twist()
        msg.linear.x = control_output_x
        msg.linear.y = control_output_y
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    except ExternalShutdownException:
        node.get_logger().info('External Shutdown Requested')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
