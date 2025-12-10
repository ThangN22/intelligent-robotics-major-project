#!/usr/bin/env python

# Provide the robot its starting pose (location and orientation) when starting simulation

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations
# reference for the units and coordinate conventions used within ROS: https://www.ros.org/reps/rep-0103.html
"""
from the reference:
By the right hand rule, the yaw component of orientation increases as the child frame rotates counter-clockwise, and for geographic poses, yaw is zero when pointing east.

Therefore, theta is zero when pointing east, and increases when turning clockwise
"""
# package (nav_msgs): https://index.ros.org/p/nav_msgs/#melodic
# package: https://wiki.ros.org/tf/Overview/Transformations
# dependencies (geometry_msgs): https://index.ros.org/p/geometry_msgs/#melodic

class Pose:
    def __init__(self):
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_pose_initialized = False

    # callback method for track_pose topic and pose node
    def callback(self, data):
        # Get X and Y coordinates
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        # get quaternion data and convert to theta for yaw specifically
        # code structure below (29-32) referencing: https://robotics.stackexchange.com/questions/53148/quaternion-transformations-in-python
        pose_data = data.pose.pose.orientation
        quaternion_points = [pose_data.x, pose_data.y, pose_data.z, pose_data.w]
        euler = tf.transformations.euler_from_quaternion(quaternion_points)
        # theta will be in radians
        yaw = euler[2]
        self.current_theta = yaw

        self.current_pose_initialized = True

if __name__ == '__main__':
    Pose()
