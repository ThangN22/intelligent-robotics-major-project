#!/usr/bin/env python

import rospy
import matrix
import node
import pose
import plan_path as path
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import radians
from sensor_msgs.msg import LaserScan
from major_project.msg import RobotStatus
from major_project.msg import PathSegment
from major_project.msg import ObjectConfidence

class MainBehavior:

    def __init__(self):
        # Set up node properties
        rospy.init_node('main_behavior')
        self.refresh_rate = 10.0
        self.rate = rospy.Rate(self.refresh_rate)
        self.turn_speed = radians(20) # degrees per second
        self.speed = 0.15 # m / s
        self.pose = pose.Pose()
        self.navigating = False
        self.turning = False

        # Publishers / Subscribers
        self.debugPub = rospy.Publisher('debug_main', String, queue_size = 5)
        self.drivePub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size = 10)
        self.statusPub = rospy.Publisher('robot_status', RobotStatus, queue_size = 1)
        self.naviPub = rospy.Publisher('navi', PathSegment, queue_size = 5)
        self.confidencePub = rospy.Publisher('detect_obj', ObjectConfidence, queue_size = 5)
        rospy.Subscriber("/barcode", String, self.barcode_callback)
        rospy.Subscriber("/odom", Odometry, self.pose_callback)
        rospy.Subscriber("scan", LaserScan, self.laser_callback)
        rospy.Subscriber("move_cmd", Twist, self.move_callback)

        # Define the node map and get path segments
        # Node locations given in meters
        self.node_0 = node.Node((0, 0), "Node0",1)
        self.node_0.add_object(1.5, -0.356, 1)
        self.node_1 = node.Node((2, 0), "Node1",1)
        self.node_1.add_object(2.356, 1.2, 1)
        self.node_2 = node.Node((2, 2), "Node2",1)
        self.node_3 = node.Node((0, 2), "Node3",1)

        # Starting node and location
        self.confidence_rating = 100
        self.current_node = self.node_0
        self.goal_node = self.node_0
        self.barcode_node = self.node_3
        self.center = 0

        self.node_list = (self.node_0, self.node_1, self.node_2, self.node_3)

        self.mat = matrix.Matrix()
        self.mat.create_matrix(len(self.node_list))
        self.mat.add_node_list(self.node_list)

        self.mat.add_edge(0,1)
        self.mat.add_edge(1,2)
        self.mat.add_edge(2,3)

        self.path_steps = path.a_star_search(self.node_0, self.node_3, self.node_list, self.mat.mat)
        self.debugPub.publish(String("end of setup"))

        # Navigate along determined path
        self.navigate()
        
        rospy.spin()

    def barcode_callback(self, data):
        # Process information read from barcode reader
        for node in self.node_list:
            # If the read QR matches name of QR
            if data.data == node.get_QR():
                self.barcode_node = node
                self.statusPub.publish(RobotStatus(node.get_QR(), self.goal_node.get_QR(), self.current_node.get_QR()))
                break

    # n is origin node, node_goal is the node the robot is traveling to, obj is the object to check
    def check_object(self, n, node_goal, obj):
        # Get the coordinates of the node and object
        node_coord = node_goal.get_coord()
        obj_coord = (obj[0], obj[1])

        # Get robot positioning
        pose_angle = self.pose.current_theta
        pose_coord = (self.pose.current_x, self.pose.current_y)

        # Determine if object is to the left of the robot
        def is_object_left():
            origin_node_coord = n.get_coord()
            x_1 = origin_node_coord[0]
            y_1 = origin_node_coord[1]
            x_2 = node_coord[0]
            y_2 = node_coord[1]
            x_3 = obj_coord[0]
            y_3 = obj_coord[1]
            return ((x_2 - x_1) * (y_3 - y_1) - (y_2 - y_1) * (x_3 - x_1)) > 0

        # distance between robot and node
        Z_len = path.calculate_distance(pose_coord, node_coord)
        if Z_len == 0:
            Z_len = 0.01

        # distance between object and node
        X_len = path.calculate_distance(obj_coord, node_coord)
        if X_len == 0:
            X_len = 0.01

        # distance between robot and object
        Y_len = path.calculate_distance(pose_coord, obj_coord)
        if Y_len == 0:
            Y_len = 0.01

        # now with all of the triangle's 3 sides' lengths known, use Cosine Rule to find theta_x
        # theta_x = arccos(((Y**2 + Z**2) - X**2) / (2*Y*Z))

        theta_x =  math.acos(((Y_len**2 + Z_len**2) - X_len**2) / (2*Y_len*Z_len))
        if is_object_left():
            theta_x = -1*theta_x
        # theta_a will be the angle between the current robot trajectory and and object.
        # math.pi - theta_x = theta_a

        return theta_x, Y_len

    # Callback for scan topic
    # Determines if objects are where expected
    def laser_callback(self, data):
        if self.turning:
            return
        # Get objects associated with the active node
        potential_objects = self.current_node.get_objects()
        if potential_objects == []:
            # Do no calculations if no objects attached to node
            return

        # Check location of object relative to robot
        for obj in potential_objects:
            angle, distance = self.check_object(self.current_node, self.goal_node, obj)
            if angle > data.angle_max or angle < data.angle_min:
                # angle outside of current view
                continue

            # Determine index within array from center value
            offset = int(angle / data.angle_increment)
            center_val = int((len(data.ranges) / 2) - offset)
            
            # Average values surrounding center_val
            width = 10
            obj_dist = 0
            data_ranges = []
            for i in range(center_val - width, center_val + width):
                if i < 0:
                    if not math.isnan(data.ranges[0]):
                        data_ranges.append(data.ranges[0])
                elif i >= len(data.ranges):
                    if not math.isnan(data.ranges[len(data.ranges) - 1]):
                        data_ranges.append(data.ranges[len(data.ranges) - 1])
                else:
                    if not math.isnan(data.ranges[i]):
                        data_ranges.append(data.ranges[i])
            
            # Determine expected object distance
            if len(data_ranges) > 0:
                obj_dist = sum(data_ranges) / len(data_ranges) - 0.45
                self.debugPub.publish(String("obj_dist = " + str(obj_dist)))
                self.debugPub.publish(String("Expected distance = " + str(distance)))
                if obj_dist < distance * 1.1 and obj_dist > distance * 0.9:
                    # Object observed within bounds
                    self.confidencePub.publish(ObjectConfidence(True, bool(obj[2])))
                else:
                    self.confidencePub.publish(ObjectConfidence(False, bool(obj[2])))
                    # Object not within bounds

    def pose_callback(self, data):
        self.pose.callback(data)

    def move_callback(self, data):
        if data == Twist():
            self.navigating = False
        else:
            self.navigating = True
        self.drivePub.publish(data)

    def debug_steps(self, steps, node_list):
        for i, n in enumerate(steps):
            node_num = node_list.index(n)
            output = "Step {}: Node_{} at {}".format(i, node_num, n.get_coord())
            self.debugPub.publish(String(output))

    def debug_segments(self, segments):
        for entry in segments:
            self.debugPub.publish(String("Angle: {} Distance: {}".format(entry[0], entry[1])))

    def get_coord_list(self, node_steps):
        coordinates = []
        for i, n in enumerate(node_steps):
            coordinates.append(n.get_coord())
        return coordinates

    def navigate(self):
        pather = path.PlanPath()
        path_segments = pather.plan(self.get_coord_list(self.path_steps), (0,0,0))
        self.debugPub.publish(String("Starting navigation"))
        # Move until we reach expected node, then go to next step
        for i, segment in enumerate(path_segments):
            self.goal_node = self.path_steps[i]
            self.debugPub.publish(String("Goal node: " + self.goal_node.get_QR()))
            self.debugPub.publish(String("Current node: " + self.current_node.get_QR()))
            # Call navigation node
            self.debugPub.publish(String(str(segment[0])))
            self.debugPub.publish(String(str(segment[1])))
            if (not segment[0] == 0.0) or (not segment[1] == 0.0):
                self.debugPub.publish(String("values valid"))
                self.naviPub.publish(PathSegment(segment[0], segment[1]))
                self.navigating = True
                self.turning = True
                while self.navigating:
                    # Processing turn from navigation node
                    pass
                self.turning = False
                while not self.navigating:
                    # Processing drive pause
                    pass
                while self.navigating:
                    # Processing distance from navigation node
                    pass
            self.current_node = self.path_steps[i] # Robot believes we have arrived at this node
            self.debugPub.publish(String("Waiting for QR to be viewed"))
            for i in range(30):
                self.rate.sleep()
            #while self.path_steps[i] != self.barcode_node:
                # Spin while waiting to find next node ID
                #pass

            self.debugPub.publish(String("Arrived at next node"))
        self.debugPub.publish(String("Reached goal node"))
        # END DEBUG

if __name__ == '__main__':
    MainBehavior()
