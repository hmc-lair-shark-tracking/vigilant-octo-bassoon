import math
import geopy.distance
import random
import timeit
import matplotlib.pyplot as plt
import numpy as np

from shapely.wkt import loads as load_wkt
from motion_plan_state import Motion_plan_state
from shapely.geometry import Polygon

class Node:
    def __init__(self, time_step, dist_traveled, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0 
        self.h = 0
        self.f = 0 
        self.time_step = time_step
        self.dist_traveled = dist_traveled

class astar:
    def __init__(self, env_info, auv_comm_msgs):
        self.path = []
        self.start = auv_comm_msgs["start"] # MPS Type
        self.obstacle_list = env_info["obstacles"]
        self.boundary = env_info["boundary"]
        self.habitat_unexplored = env_info["habitats"]
        self.habitat_explored = []
        self.visited_nodes = []
        self.velocity = auv_comm_msgs["velocity"]
        self.plantime = auv_comm_msgs["plan_time"]

    def euclidean_dist(self, point1, point2):
        """
        Calculate the distance square between two points
        Parameter:
            point1 - a position tuple: (x, y)
            point2 - a position tuple: (x, y)
        """

        dx = abs(point1[0]-point2[0])
        dy = abs(point1[1]-point2[1])

        return math.sqrt(dx*dx+dy*dy)

    def same_side(self, p1, p2, a, b):
        """
        Check if p1 and p2 are on the same side
        Parameters:
            p1, p2, a, b -- a position tuple (x, y)
        
        Returns:
            True if p1 and p2 are on the same side
            False if p1 and p2 are on different sides
        """

        cp1 = np.cross(np.asarray(b)-np.asarray(a), np.asarray(p1)-np.asarray(a))
        cp2 = np.cross(np.asarray(b)-np.asarray(a), np.asarray(p2)-np.asarray(a))
        if np.dot(cp1, cp2) >= 0:
            return True
        else:
            return False
    
    def point_in_triangle(self, p, a, b, c):
        """
        Check if p is inside the triangle formed by connecting a, b, c
        Parameters:
            p, a, b, c -- a position tuple (x, y)
        Returns:
            True if p is inside the triangle
            False if p is outside of the triangle
        """

        if self.same_side(p, a, b, c) and self.same_side(p, b, a, c) and self.same_side(p, c, a, b):
            return True
        else:
            return False
    
    def within_bounds(self, boundary_list, position):
        """
        Check if the input position is within the boundary defined by the boundary_list
        Paramters: 
            boundary_list -- a list of Motion_plan_state objects that define the corners of the ROI
            position -- a position tuple (x, y) 
        Returns:
            True if the position is inside the bounds
            False if the position is outside of the bounds
        """
        poly_list = []  
        for corner in boundary_list: 
            poly_list.append([corner.x, corner.y])
        centroid = Polygon(poly_list).centroid.coords
        for index in range(len(poly_list)):
            if index != len(poly_list)-1:
                if self.point_in_triangle(position, poly_list[index], poly_list[index+1], centroid):
                    return True 
            else:
                if self.point_in_triangle(position, poly_list[len(poly_list)-1], poly_list[0], centroid):
                    return True
        return False


    def curr_neighbors(self, current_node, boundary_list): 

        """
        Return a list of position tuples that are close to the current point
        Parameter:
            current_node: a Node object 
        """
        adjacent_squares = [(0, -25), (0, 25), (-25, 0), (25, 0), (-25, -25), (-25, 25), (25, -25), (25, 25)]
        current_neighbors = []
        for new_position in adjacent_squares:
            node_x_pos = current_node.position.x + new_position[0]
            node_y_pos = current_node.position.y + new_position[1]
            node_position = (node_x_pos, node_y_pos)
            # check if it's within the boundary
            if self.within_bounds(boundary_list, node_position):
                current_neighbors.append(node_position)
        return current_neighbors
    
    def update_habitat_coverage(self, current_node, habitat_unexplored, habitat_explored):
        """
        Check if the current node covers a habitat (either explored or unexplored);
        then update the habitat_open_list that holds all unexplored habitats
        and update the habitat_closed_list that holds all explored habitats 
        Parameter:
            current_node: a Node object 
            habitat_open_list: a list of Motion_plan_state objects
            habitat_closed_list: a list of Motion_plan_state objects
        """
        for index, item in enumerate(habitat_unexplored):
            dist = math.sqrt((current_node.position[0]-item.x) **2 + (current_node.position[1]-item.y) **2)
            if dist <= item.size:
                self.habitat_unexplored.pop(index)
                self.habitat_explored.append(item)
    
    def num_habitat_within_R(self, current_node, R):
        dist_sum = 0
        i = 0 # Indexing the habitat 
        while (2 * dist_sum < R and i < len(self.habitat_unexplored)):
            dist_sum += self.euclidean_dist((self.habitat_unexplored[i].x, self.habitat_unexplored[i].y), \
                                            (current_node.position.x, current_node.position.y))
            i += 1 # Update index
        return i+1 # The number of habitats within range R

    def astar(self): 
        """
        Find the optimal path from start to goal avoiding given obstacles 
        Parameter: 
            obs_lst - a list of motion_plan_state objects that represent obstacles 
            start - a tuple of two elements: x and y coordinates
            goal - a tuple of two elements: x and y coordinates
        """

        start_node = Node(0, 0, None, (self.start.x, self.start.y))
        start_node.g = start_node.h = start_node.f = 0
        open_list = [] # hold neighbors of the expanded nodes
        closed_list = [] # hold all the exapnded nodes

        open_list.append(start_node)

        while len(open_list) > 0:
            current_node = open_list[0] # initialize the current node
            current_index = 0

            for index, item in enumerate(open_list): # find the current node with the smallest f
                if item.f < current_node.f:
                    current_node = item
                    current_index = index
            
            # Update time_step of the current node
            open_list.pop(current_index)
            closed_list.append(current_node) 
            if current_node.time_step >= self.plantime: # terminating condition
                current = current_node
                while current is not None: # backtracking to find the d 
                    self.path.append(current.position)
                    current = current.parent
                path_mps = [] 
                for point in self.path:
                    path_mps.append(Motion_plan_state(point[0], point[1]))
                trajectory = path_mps[::-1]
                return trajectory

            current_neighbors = self.curr_neighbors(current_node, self.boundary)
            children = []
            current_pos = (current_node.position.x, current_node.position.y)

            for neighbor in current_neighbors: # create new node
                curr_dist = current_node.dist_traveled + self.euclidean_dist(current_pos, neighbor)
                curr_time_step = int(curr_dist/self.velocity)

                new_node = Node(curr_time_step, curr_dist, current_node, neighbor) 
                self.update_habitat_coverage(new_node, self.habitat_unexplored, self.habitat_explored)
                children.append(new_node)

            for child in children: 
                if child in closed_list:
                    continue
                
                # The number of habitats already visiteed
                child.g = len(self.habitat_explored)
                # The number of unexplored habitats within range
                child.h = self.num_habitat_within_R(current_node, abs(self.plantime - child.time_step) * self.velocity)
                child.f = child.g + child.h

                # check if child exists in the open list and have bigger g 
                for open_node in open_list:
                    if child == open_node and child.g < open_node.g:
                        continue
            
                if child.position not in self.visited_nodes:
                    open_list.append(child)
                    self.visited_nodes.append(child)