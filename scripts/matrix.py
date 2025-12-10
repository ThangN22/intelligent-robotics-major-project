#!/usr/bin/env python
# create matrix data structure for map representation
# https://www.geeksforgeeks.org/dsa/adjacency-matrix/
# https://www.geeksforgeeks.org/python/python-using-2d-arrays-lists-the-right-way/
import sys
import node
import plan_path as path


class Matrix:
    mat =[0][0]
    node_list = [0]
    def create_matrix(self, node_length):
        # set all weights to zero by default
        # weights are tuples to support the (angle, distance) data structure
        # plan_path.plan() in plan_path.py returns such tuples, but it currently acts as a piece in the task pipeline defined in project two alternative. Further modification is needed for use in matrix.py
        self.mat = [[(0.0, 0.0) for i in range(node_length)] for j in range(node_length)]
        self.create_node_list(node_length)
        self.print_node_list()

    def create_node_list(self, node_length):
        self.node_list = [0 for i in range(node_length)]

    def add_node(self, node, idx):
        self.node_list[idx] = node

    def add_node_list(self, node_list):
        self.node_list = node_list

    def print_node_list(self):
        for i, nod in enumerate(self.node_list):
            if isinstance(nod, node.Node):
                print("  node_{}: {}".format(i, nod.get_coord()))
            else:
                print("  node_{}: Not initialized".format(i))

    # add_edge will take in two nodes. The node data structure contains its coordinates on the map. These coordinates will be used to calculate the distance between two nodes as well as the turning angle required. The edges will be added in both directions with different angles to account for the two directions.
    def add_edge(self, i, j):
        starting_node = self.node_list[i]
        ending_node = self.node_list[j]
        if isinstance(starting_node, node.Node) and isinstance(ending_node, node.Node):
            starting_coord = starting_node.get_coord()
            ending_coord = ending_node.get_coord()
            weight = path.calculate_distance(starting_coord, ending_coord)
            starting_angle = path.determine_angle(starting_coord, ending_coord)
            ending_angle = path.determine_angle(ending_coord, starting_coord)
            forward_tuple = (starting_angle, weight)
            backward_tuple = (ending_angle, weight)
            self.mat[i][j] = forward_tuple
            self.mat[j][i] = backward_tuple
        elif not isinstance(ending_node, node.Node):
            print("ERROR: ending node at index {} not initiated".format(j))
        else:
            print("ERROR: starting node at index {} not initiated".format(i))

    def printCustom(self):
        for row in self.mat:
            print(row)

    def print_simple(self):
        for row in self.mat:
            row_list = []
            for tup in row:
                if tup[1] > 0.0:
                    row_list.append('1')
                else:
                    row_list.append('0')
            print(row_list)


# TODO: link nodes from node.py to matrices in this here matrix.py
