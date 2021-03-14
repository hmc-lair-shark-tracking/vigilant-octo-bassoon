import math
import random
import time
import csv
import numpy as np
from shapely.geometry import Polygon, Point
from motion_plan_state import Motion_plan_state

class RRT:
    def __init__(self, env_info):
        """
        Initialize RRT class given AUV common messages and environmental information

        Parameters:
            auv_comm_msgs - Python dictionary, certain information about the AUV
                (e.g. start, goal, plantime)
            env_info - Python dictionary, contain information neccessary for the planner
                (e.g. boundary, obstacles, habitats)
        """
        self.boundary_poly = env_info["boundary"]
        self.obstacle_list = env_info["obstacles"]
        self.habitats = env_info["habitats"]
        self.path = []
        self.time_bin = {}

        # Returned if minimum path length is not achieved within maximum iteration
        self.last_path = []

        # Initialize parameters 
        self.exp_rate = 1
        self.dist_to_end = 2
        self.diff_max = 0.5
        self.freq = 30

    def splitPath(self, path, shark_interval, traj_time):
        n_expand = math.floor( traj_time[1] / shark_interval)
        res = {}
        start = traj_time[0]
        for i in range(n_expand):
            res[(start + i * shark_interval, start+(i + 1) * shark_interval)] = []
        
        for point in path:
            for time, arr in res.items():
                if point.traj_time_stamp >= time[0] and point.traj_time_stamp <= time[1]:
                    arr.append(point)
                    break
        return res

    def get_closest_mps_time(self, ran_time, mps_list):
        while len(mps_list) > 3:
            left = mps_list[len(mps_list)//2 - 1]
            left_diff = abs(left.plan_time_stamp - ran_time)
            right = mps_list[len(mps_list)//2 + 1]
            right_diff = abs(right.plan_time_stamp - ran_time)

            index =  len(mps_list)//2 
            if left_diff >= right_diff:
                mps_list = mps_list[index:]
            else:
                mps_list = mps_list[:index]
        
        return mps_list[0]

    def get_closest_mps(self, ran_mps, mps_list):
        min_dist, _ = self.get_distance_angle(mps_list[0],ran_mps)
        closest_mps = mps_list[0]
        for mps in mps_list:
            dist, _ = self.get_distance_angle(mps, ran_mps)
            if dist < min_dist:
                min_dist = dist
                closest_mps = mps
        return closest_mps

    def get_distance_angle(self, start_mps, end_mps):
        dx = end_mps.x-start_mps.x
        dy = end_mps.y-start_mps.y
        #dz = end_mps.z-start_mps.z
        dist = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy,dx)
        return dist, theta

    def check_collision(self, mps, obstacleList):

        if mps is None:
            return False

        dList = []
        for obstacle in obstacleList:
            for point in mps.path:
               d, _ = self.get_distance_angle(obstacle, point)
               dList.append(d) 

            if min(dList) <= obstacle.size:
                return False  # collision
        
        for point in mps.path:
            point = Point(point.x, point.y)
            if not point.within(self.boundary_poly):
                return False

        return True  # safe

    def get_random_mps(self, size_max=15):
        x_min, y_min, x_max, y_max= self.boundary_poly.bounds

        ran_x = random.uniform(x_min, x_max)
        ran_y = random.uniform(y_min, y_max)
        ran_theta = random.uniform(-math.pi, math.pi)
        ran_size = random.uniform(0, size_max)
        mps = Motion_plan_state(ran_x, ran_y, theta=ran_theta, size=ran_size)
        #ran_z = random.uniform(self.min_area.z, self.max_area.z)
        
        return mps

    def generate_final_course(self, mps):
        path = [mps]
        mps = mps
        while mps.parent is not None:
            reversed_path = reversed(mps.path)
            for point in reversed_path:
                path.append(point)
            mps = mps.parent
        #path.append(mps)

        return path

    def steer(self, mps, dist_to_end, diff_max, freq, min_dist, velocity=1, traj_time_stamp=False):
        #dubins library
        '''new_mps = Motion_plan_state(from_mps.x, from_mps.y, theta = from_mps.theta)
        new_mps.path = []
        q0 = (from_mps.x, from_mps.y, from_mps.theta)
        q1 = (to_mps.x, to_mps.y, to_mps.theta)
        turning_radius = 1.0
        path = dubins.shortest_path(q0, q1, turning_radius)
        configurations, _ = path.sample_many(exp_rate)
        for configuration in configurations:
            new_mps.path.append(Motion_plan_state(x = configuration[0], y = configuration[1], theta = configuration[2]))
        new_mps.path.append(to_mps)
        dubins_path = new_mps.path
        new_mps = dubins_path[-2]
        new_mps.path = dubins_path'''
        new_mps = Motion_plan_state(mps.x, mps.y, theta = mps.theta, plan_time_stamp=time.time()-self.t_start, traj_time_stamp=mps.traj_time_stamp, length=mps.length)

        new_mps.path = [mps]

        '''if dist_to_end > d:
            dist_to_end = dist_to_end / 10
        n_expand = math.floor(dist_to_end / exp_rate)'''
        n_expand = random.uniform(0, freq)
        n_expand = math.floor(n_expand/1)

        for _ in range(n_expand):
            #setting random parameters
            dist = random.uniform(0, dist_to_end)#setting random range
            diff = random.uniform(-diff_max, diff_max)#setting random range
            if abs(dist) > abs(diff):

                s1 = dist + diff
                s2 = dist - diff
                radius = (s1 + s2)/(-s1 + s2)
                phi = (s1 + s2)/ (2 * radius)
                
                ori_theta = new_mps.theta
                new_mps.theta += phi
                delta_x = radius * (math.sin(new_mps.theta) - math.sin(ori_theta))
                delta_y = radius * (-math.cos(new_mps.theta) + math.cos(ori_theta))
                new_mps.x += delta_x
                new_mps.y += delta_y
                velocity_temp = random.uniform(0, 2*velocity)
                movement = math.sqrt(delta_x ** 2 + delta_y ** 2)
                new_mps.traj_time_stamp += movement / velocity_temp
                new_mps.length += movement
                if movement >= min_dist:
                    new_mps.path.append(Motion_plan_state(new_mps.x, new_mps.y, v=velocity_temp, theta=new_mps.theta, traj_time_stamp=new_mps.traj_time_stamp, plan_time_stamp=time.time()-self.t_start, length=new_mps.length))

            #d, theta = self.get_distance_angle(new_mps, to_mps)

        '''d, _ = self.get_distance_angle(new_mps, to_mps)
        if d <= dist_to_end:
            new_mps.path.append(to_mps)'''

        #new_node.parent = from_node
        new_mps.path[0] = mps

        return new_mps

    def connect_to_goal_curve_alt(self, mps, exp_rate, goal):
        new_mps = Motion_plan_state(mps.x, mps.y, theta=mps.theta, traj_time_stamp=mps.traj_time_stamp)
        theta_0 = new_mps.theta
        _, theta = self.get_distance_angle(mps, goal)
        diff = theta - theta_0
        diff = self.angle_wrap(diff)
        if abs(diff) > math.pi / 2:
            return

        #polar coordinate
        r_G = math.hypot(goal.x - new_mps.x, goal.y - new_mps.y)
        phi_G = math.atan2(goal.y - new_mps.y, goal.x - new_mps.x)

        #arc
        phi = 2 * self.angle_wrap(phi_G - new_mps.theta)
        radius = r_G / (2 * math.sin(phi_G - new_mps.theta))

        length = radius * phi
        if phi > math.pi:
            phi -= 2 * math.pi
            length = -radius * phi
        elif phi < -math.pi:
            phi += 2 * math.pi
            length = -radius * phi
        new_mps.length += length

        ang_vel = phi / (length / exp_rate)

        #center of rotation
        x_C = new_mps.x - radius * math.sin(new_mps.theta)
        y_C = new_mps.y + radius * math.cos(new_mps.theta)

        n_expand = math.floor(length / exp_rate)
        for i in range(n_expand+1):
            new_mps.x = x_C + radius * math.sin(ang_vel * i + theta_0)
            new_mps.y = y_C - radius * math.cos(ang_vel * i + theta_0)
            new_mps.theta = ang_vel * i + theta_0
            new_mps.path.append(Motion_plan_state(new_mps.x, new_mps.y, theta = new_mps.theta, plan_time_stamp=time.time()-self.t_start))
        
        return new_mps

    def plan_trajectory(self, auv_comm_msgs):
        """
        rrt path planning
        animation: flag for animation on or off
        """
        bin_interval = 5
        max_plan_time = auv_comm_msgs["plan_time"]
        velocity = auv_comm_msgs["velocity"]
        start = auv_comm_msgs["start"] # MPS type
        goal = auv_comm_msgs["goal"]
        max_traj_time = 200.0
        plan_time = True
        traj_time_stamp = False 

        self.mps_list = [start]
        self.t_start = time.time()
        t_end = time.time() + max_plan_time
        if traj_time_stamp:
            time_expand = math.ceil(max_traj_time / bin_interval)
            for i in range(1, time_expand + 1):
                self.time_bin[bin_interval * i] = []
            self.time_bin[bin_interval].append(start)
        while time.time()<t_end:
            #find the closest motion_plan_state by generating a random time stamp and 
            #find the motion_plan_state whose time stamp is closest to it
            if plan_time:
                if traj_time_stamp:
                    ran_bin = int(random.uniform(1, time_expand + 1))
                    while self.time_bin[bin_interval * ran_bin] == []:
                        ran_bin = int(random.uniform(1, time_expand + 1))
                    ran_index = int(random.uniform(0, len(self.time_bin[bin_interval * ran_bin])))
                    closest_mps = self.time_bin[bin_interval * ran_bin][ran_index]
                else:
                    ran_time = random.uniform(0, max_plan_time * self.freq)
                    closest_mps = self.get_closest_mps_time(ran_time, self.mps_list)
                    if closest_mps.traj_time_stamp > max_traj_time:
                        continue
            #find the closest motion_plan_state by generating a random motion_plan_state
            #and find the motion_plan_state with smallest distance
            else:
                ran_mps = self.get_random_mps()
                closest_mps = self.get_closest_mps(ran_mps, self.mps_list)
                if closest_mps.traj_time_stamp > max_traj_time:
                    continue
            
            new_mps = self.steer(closest_mps, self.dist_to_end, self.diff_max, self.freq, velocity, traj_time_stamp)
            if self.check_collision(new_mps, self.obstacle_list):
                print(new_mps)
                new_mps.parent = closest_mps
                self.mps_list.append(new_mps)
                #add to time stamp bin
                if traj_time_stamp:
                    curr_bin = (new_mps.traj_time_stamp // bin_interval + 1) * bin_interval
                    if curr_bin > max_traj_time:
                        #continue
                        self.time_bin[curr_bin] = []
                    self.time_bin[curr_bin].append(new_mps)
                
            final_mps = self.connect_to_goal_curve_alt(self.mps_list[-1], self.exp_rate, goal)
            if self.check_collision(final_mps, self.obstacle_list):
                final_mps.parent = self.mps_list[-1]
                path = self.generate_final_course(final_mps)
                return path
        
        return None  # cannot find path
    