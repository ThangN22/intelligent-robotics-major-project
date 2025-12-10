#!/usr/bin/env python

import rospy
import pose
import plan_path as path
from std_msgs.msg import Int32
from std_msgs.msg import String
from major_project.msg import RobotStatus
from major_project.msg import PathSegment
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians

class Navigation:
    def __init__(self):
        rospy.init_node('navigation')
        self.rate = rospy.Rate(10)
        self.pose = pose.Pose()
        self.turn_speed = radians(20) # degrees per second
        self.angle_offset = 0.1
        self.speed = 0.15 # m / s
        self.pub = rospy.Publisher('move_cmd', Twist, queue_size = 10)
        rospy.Subscriber('navi', PathSegment, self.callback)
        rospy.Subscriber("/odom", Odometry, self.pose_callback)
        
        rospy.spin()

    # Navigation command passed to node
    # Contains turn angle and distance to travel
    def callback(self, data):
        start_angle = self.pose.current_theta
        start_location = (self.pose.current_x, self.pose.current_y)

        twist = Twist()
        if data.turn_angle > 0:
            twist.angular.z =  self.turn_speed
        else:
            twist.angular.z = -1 * self.turn_speed

        while abs(start_angle - self.pose.current_theta) < abs(data.turn_angle) - self.angle_offset:
                self.pub.publish(twist)
                self.rate.sleep()

        for i in range(10):
            self.pub.publish(Twist())
            self.rate.sleep()
        
        twist.angular.z = 0
        twist.linear.x = self.speed
        while path.calculate_distance(start_location, [self.pose.current_x, self.pose.current_y]) < data.distance:
            self.pub.publish(twist)
            self.rate.sleep()

        for i in range(10):
            self.pub.publish(Twist())
            self.rate.sleep()

    def pose_callback(self, data):
        self.pose.callback(data)

if __name__ == '__main__':
    Navigation()

