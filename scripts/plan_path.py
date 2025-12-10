#!/usr/bin/env python

import math
from collections import deque
import heapq
# TODO: explain importance of heapq

import node

class PlanPath(object):
    def __init__(self):
        pass
    # given ordered waypoints from plan_task.plan(), generate lists of turn angles and straight-line distances for the robot
    def plan(self, ordered_waypoints, robot_position):
        angle_and_distance = []
        current_position = [robot_position[0], robot_position[1], robot_position[2]]
        for waypoint in ordered_waypoints:
            turn_angle = determine_angle(current_position, waypoint)
            distance = calculate_distance((current_position[0], current_position[1]), waypoint)
            angle_and_distance.append((turn_angle, distance))
            # Update robot's xy coordinates and turn angle for next iteration of for loop
            current_position[0] = waypoint[0]
            current_position[1] = waypoint[1]
            current_position[2] += turn_angle
        return angle_and_distance

def determine_angle(robot_position, waypoint):
    # https://stackoverflow.com/questions/64782652/shifting-angles-while-using-atan2
    # To compute angle clockwise from X-axis, compute arctan from (Y2- Y1, X2 - X1)
    X1 = robot_position[0]
    Y1 = robot_position[1]
    X2 = waypoint[0]
    Y2 = waypoint[1]
    # use atan2 for correct results across all 4 quadrants
    theta = math.atan2(Y2 - Y1, X2 - X1)
    # desired angle will be how much the robot will have to turn from its yaw to point toward the waypoint IF a robot's angle is provided
    if len(robot_position) == 3:
        desired_angle = theta - robot_position[2]
    else:
        desired_angle = theta
    # For the robot to not turn more than 180 degrees in either direction to head to the desired angle, normalize theta to be within negative pi (-180) and pi (180)
    while desired_angle > math.pi:
        desired_angle -= 2 * math.pi # 2pi = 360 degrees
    while desired_angle < -math.pi:
        desired_angle += 2 * math.pi
    return desired_angle

def calculate_distance(point1, point2):
    # use distance formula to calculate distance between two points
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# A* Search
# https://www.datacamp.com/tutorial/a-star-algorithm
def a_star_search(starting_node, goal_node, node_list, matrix):

    def heuristic(node_idx):
        current_coord = node_list[node_idx].get_coord()
        goal_coord = goal_node.get_coord()
        return calculate_distance(current_coord, goal_coord)

    start_idx = node_list.index(starting_node)
    goal_idx = node_list.index(goal_node)

    # to-be evaluated
    open_list = [(heuristic(start_idx), start_idx)]

    # already evaluated
    closed_list = []

    # parents (dictionaries)
    parents = {}

    g_score = {i: float('inf') for i in range(len(node_list))}
    g_score[start_idx] = 0.0

    while open_list:
        # get node with smallest f = (g + h) score
        current_f, current_idx = heapq.heappop(open_list)

        if current_idx == goal_idx:
            # Reconstruct path
            path_indices = []
            while current_idx in parents:
                path_indices.append(current_idx)
                # set current_idx to be its parents (dictionary), so parents[current_idx] uses the current_idx as a key to return the parent value
                current_idx = parents[current_idx]
            # add start index at the end and reverse so that it is at the beginning
            path_indices.append(start_idx)
            path_indices.reverse()

            # Convert indices to node objects
            node_path_list = []
            for i in path_indices:
                node_path_list.append(node_list[i])
            return node_path_list

        # Skip if already visited
        if current_idx in closed_list:
            continue
        # Otherwise add to closed_list
        closed_list.append(current_idx)

        # Explore neighbors
        for neighbor_idx in range(len(node_list)):
            # Check if there's an edge by between current node and neighboring node by checking for distance (distance > 0)
            edge_data = matrix[current_idx][neighbor_idx]
            distance = edge_data[1]  # (angle, distance) tuple

            if distance > 0:
                # Calculate g_score
                tentative_g = g_score[current_idx] + distance

                # If this path's tentative g_score is better than any previous one
                if tentative_g < g_score[neighbor_idx]:
                    # Then update the best path to neighbor
                    parents[neighbor_idx] = current_idx
                    g_score[neighbor_idx] = tentative_g
                    f_score = tentative_g + heuristic(neighbor_idx)

                    # Add to open_list
                    heapq.heappush(open_list, (f_score, neighbor_idx))

    # Return none if no path found
    return None

def print_a_star_results(result, node_list):
    for i, n in enumerate(result):
        node_num = node_list.index(n)
        print("  Step {}: Node_{} at {}".format(i, node_num, n.get_coord()))


if __name__ == '__main__':
    PlanPath()


