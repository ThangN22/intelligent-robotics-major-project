#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from major_project.msg import RobotStatus
from major_project.msg import ObjectConfidence
from geometry_msgs.msg import Twist

# Confidence node that handles processing how confident the robot is in its location
class Confidence:
    def __init__(self):
        # Initalize variables and rospy nodes
        rospy.init_node('confidence')
        self.rate = rospy.Rate(2)
        self.pub = rospy.Publisher('confidence', Int32, queue_size = 5)
        rospy.Subscriber('robot_status', RobotStatus, self.callback)
        rospy.Subscriber('detect_obj', ObjectConfidence, self.obj_callback)
        rospy.Subscriber('cmd_vel_mux/input/navi', Twist, self.move_callback)

        self.confidence = 100 # location confidence value
        self.confidence_rate = 0.05 # rate at which confidence decreases as we move

        while not rospy.is_shutdown():
            # Publish confidence rate continously
            self.pub.publish(Int32(self.confidence))
            self.rate.sleep()
        rospy.spin()

    # Process recieved robot status messages and update confidence if detect correct barcode
    def callback(self, data):
        if data.barcode_node == data.current_node:
            # Barcode confirms we are where we think we are
            # Increase confidence
            self.confidence = 100

    def move_callback(self, data):
        # Movement message recieved, decrease confidence slowly
        if data == Twist():
            return
        self.confidence = self.confidence - 1 * self.confidence_rate

    def obj_callback(self, data):
        # Laser scan detected object in correct location
        if data.detected:
            # Increase confidence by a higher value if suffering from low surety
            if self.confidence < 30:
                self.confidence = self.confidence + self.confidence_rate * 2
            else:
                # Increase confidence by typical value to offset odometry loss
                self.confidence = min(self.confidence + self.confidence_rate * 0.5, 100.0)
        else:
            # Expected object is missing
            if data.moveable:
                # Missing moveable object
                self.confidence = self.confidence - self.confidence_rate * 1
            else:
                # Missing immovable object
                self.confidence = self.confidence - self.confidence_rate * 5
        pass

if __name__ == '__main__':
    Confidence()
