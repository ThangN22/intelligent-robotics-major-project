#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# Behavior management node
# Subscribes to behavior nodes and forwards commands to turtlebot

class Behavior:
    def __init__(self):
        # Create supression variables and subscribers
        rospy.init_node('behavior')
	# Drive Forward
        self._df_supressed = False
	# Random Turn
        self._rt_supressed = False
	# Avoid
        self._av_supressed = False
	# Escape
        self._es_supressed = False
	# Bumper
        self._ky_supressed = False

        self._rate = rospy.Rate(10)

        rospy.Subscriber("turn_cmd", Twist, self.turn_callback)
        rospy.Subscriber("move_cmd", Twist, self.df_callback)
        rospy.Subscriber("avoid_cmd", Twist, self.avoid_callback)
        rospy.Subscriber("escape_cmd", Twist, self.escape_callback)
        rospy.Subscriber("keyboard_cmd", Twist, self.keyboard_callback, queue_size = 1)
        rospy.Subscriber("bumper_cmd", Twist, self.bumper_callback, queue_size = 1)

        # Publisher to turtlebot control
        self._pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 1)

        rospy.spin()
    
    # Callback for drive forward node
    # Supresses no nodes
    # Forwards command if no other nodes are active
    def df_callback(self, data):
        if self._df_supressed == False and self._rt_supressed == False and self._av_supressed == False and self._es_supressed == False:
            if self._ky_supressed == False:
                self._pub.publish(data)
            else:
                # Call bumper_movement if bumper is pressed
                self.bumper_movement(data)

    # Callback for random turning node
    # Supresses drive_forward node
    # Overridden by avoid, escape, keyboard, and bumper nodes
    def turn_callback(self, data):
        # Empty Twist marks end of turn movement
        if data == Twist():
            self._df_supressed = False
        elif self._rt_supressed == False and self._av_supressed == False and self._es_supressed == False:
            self._df_supressed = True

            if self._ky_supressed == False:
                self._pub.publish(data)
            else:
                # Call bumper_movement if bumper is pressed
                self.bumper_movement(data)

    # Callback for avoiding node
    # Supresses random turning node
    # Overridden by escape, keyboard, and bumper nodes
    def avoid_callback(self, data):
        # Empty Twist marks end of avoiding movement
        if data == Twist():
            self._rt_supressed = False
        elif self._av_supressed == False and self._es_supressed == False:
            self._rt_supressed = True
            if self._ky_supressed == False:
                self._pub.publish(data)
            else:
                self.bumper_movement(data)

    # Callback for escaping node
    # Supresses avoid node
    # Overridden keyboard and bumper nodes
    def escape_callback(self, data):
        # Empty Twist marks end of escape movement
        if data == Twist():
            self._av_supressed = False
        elif self._es_supressed == False:
            self._av_supressed = True
            if self._ky_supressed == False:
                self._pub.publish(data)
            else:
                self.bumper_movement(data)

    # Callback for keyboard node
    # Supresses escape node
    # Overridden by bumper node
    def keyboard_callback(self, data):
        # Empty Twist marks end of keyboard input
        if data == Twist():
            self._es_supressed = False
        else:
            self._es_supressed = True
            if self._ky_supressed == False:
                self._pub.publish(data)
            else:
                self.bumper_movement(data)

    # Callback for bumper node
    # Supresses keyboard node
    # Overridden by no nodes
    def bumper_callback(self, data):
        self._ky_supressed = True
        # Empty Twist marks bumper no longer pressed
        if data == Twist():
            self._ky_supressed = False

    # Movement command for active bumper press to halt if collision(s) are detected by bumper(s)
    def bumper_movement(self, data):
	start_time = rospy.Time.now()
	# Publishing empty Twist and sleep in a timed while-loop
	while (rospy.Time.now().secs - start_time.secs) < 2.0:
        	self._pub.publish(Twist())
        	self._rate.sleep()
        

if __name__ == '__main__':
    Behavior()
