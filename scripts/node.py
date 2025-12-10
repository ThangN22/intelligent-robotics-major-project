#!/usr/bin/env python
# create node data structure for map representation



class Node:
    def __init__(self, coord, name, radius):
        self.set_coord(coord)
        self.set_qr(name)
        self.objects = []
        self.radius = radius

    def create_node(self, coord):
        self.set_coord(coord)
        return 0

    def set_coord(self, coord):
        self.coordinate = coord

    def get_coord(self):
        return self.coordinate

    def set_qr(self, name):
        self.name = name
        return 0

    def get_QR(self):
        return self.name
    
    def add_object(self, obj_x, obj_y, movable):
        # Objects in form [coords, movable (0 no, 1 yes)]
        self.objects.append([obj_x, obj_y, movable])
        return 0

    def get_objects(self):
        return self.objects
    
    def get_radius(self):
        return self.radius

    # TODO: implement code
