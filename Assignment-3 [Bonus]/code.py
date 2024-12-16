#!/usr/bin/env python3
import rclpy
import math
from math import atan, atan2, sqrt, sin, cos, pi
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.rad=1.0
        self.v=2.0
        self.ld=0.05
        self.x=0
        self.y=0
        self.lda=0
        self.theta=0
        self.dt=0.0005
        self.pos_pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.control_timer=self.create_timer(self.dt, self.control_loop)
        self.start_time=self.get_clock().now()
    
    def control_loop(self):
        now=self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9
        if elapsed_time > 15.0:
        	twist_msg = Twist()
        	twist_msg.linear.x=0.0
        	twist_msg.angular.z=0.0
        	self.pos_pub.publish(twist_msg)
        else:
        	self.theta=atan2(self.y,self.x)
        	self.lda=self.theta+((self.v)*(self.dt)/(self.rad))
        	x_ahead=self.rad*cos(self.lda)
        	y_ahead=self.rad*sin(self.lda)
        	dx=x_ahead-self.x
        	dy=y_ahead-self.y
        	self.x+=2*cos(self.theta)*self.dt
        	self.y+=2*sin(self.theta)*self.dt
        	alpha=atan2(dy,dx)-self.theta
        	phi=atan(2*(0.0333333333333333333333/self.ld)*sin(alpha))
        	self.theta+=phi
        	twist_msg=Twist()
        	twist_msg.linear.x=2.0
        	twist_msg.angular.z=(phi/self.dt)
        	self.pos_pub.publish(twist_msg)


rclpy.init()
controller=PurePursuitController()
rclpy.spin(controller)
controller.destroy_node()
rclpy.shutdown()
