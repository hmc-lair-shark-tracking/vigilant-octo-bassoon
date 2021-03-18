from ParticleFilter import ParticleFilter
from motionPlanners.MotionPlanner import MotionPlanner
import math
import numpy as np

def angle_wrap(ang):
    """
    Take an angle in radians and set it between the range of -pi to pi
    (USE ANY TIME THERE IS AN ANGLE CALCULATION)

    Parameter:
        ang - angle in radians

    Return:
        the wrapped/transformed angle (from -pi to pi)
    """
    if -math.pi <= ang <= math.pi:
        return ang
    elif ang > math.pi: 
        ang += (-2 * math.pi)
        return angle_wrap(ang)
    elif ang < -math.pi: 
        ang += (2 * math.pi)
        return angle_wrap(ang)



class Auv:
    def __init__(self, init_x, init_y, init_z, init_theta, init_v, init_w, planner_key_list, env_info):
        """
        Initialilze an AUV class given the initial auv position and other configurations

        Parameters:
            init_x - initial x position
            init_y - initial y position
            init_z - initial z position
            init_theta - initial theta position
            init_v - nonzero, initial linear velocity
                Warning: currently, no way to change the linear velocity
            init_w - inital angular velocity
            planner_key_list - Python list, indicate which planner(s) to use in this auv
                TODO: check with MotionPlanner code - is this format ok?
            env_info - Python dictionary, contain information neccessary for the planner
                (e.g. boundary, obstacles, habitats)
                TODO: check with MotionPlanner code - is this format ok?
        """
        self.state = MotionPlanState(init_x, init_y, init_z, init_theta, init_v, init_w)

        self.particle_filter = ParticleFilter()

        self.motion_planner = MotionPlanner(planner_key_list, env_info)

        self.curr_traj_pt_index = 0
        self.TRAJ_LOOK_AHEAD_TIME = 0.5
        
        # for plotting
        self.x_list_plot = [init_x]
        self.y_list_plot = [init_y]
        self.z_list_plot = [init_z]


    def run_auv_control_loop(self, shark_measurements, auv_comm_msgs, curr_time, delta_t):
        """
        Run one iteration of the auv control loop
            (Called by the worldSim class)

        Parameters:
            shark_measurements - Python list of MotionPlanState, represent the shark measurements
                TODO: check with worldSim - what format?
            auv_comm_msgs - Python list of dictionaries, representing the information sent by other auvs
                TODO: check with MotionPlanner - is this format ok?
            curr_time - in seconds, current time in the worldSim
            delta_t - in seconds, time step
        
        Return:
            dictionary, representing the communication message sent from the auv
        """
        shark_state_list = self.calc_shark_state(shark_measurements)
        shark_state_estimates, shark_estate_estimation_err = self.particle_filter.estimate_shark_state(
                                                                    self.state, 
                                                                    shark_state_list, 
                                                                    delta_t)
        auv_trajectory, new_trajectory = self.motion_planner.plan_trajectory(self, shark_state_estimates, auv_comm_msgs)
        auv_control_signals = self.control_based_on_traj(auv_trajectory, new_trajectory, curr_time)
        
        self.update_auv_state(auv_control_signals, delta_t)

        return self.create_auv_msg(shark_state_estimates, auv_trajectory)


    def calc_shark_state(self, shark_measurements):
        """
        Based on the shark_measurements, calculate additional shark information (related to the auv position)
            which would get used in particle filter

        Parameter:
            shark_measurements - Python list of MotionPlanState, represent the shark measurements
                TODO: check with worldSim - what format?
        
        Return: 
            Python list of lists, each element is the measurement for a shark, 
                each element's format: [x_shark, y_shark, z_shark_range, z_shark_bearing, shark_id]
        """
        shark_state_list = []
        for shark in shark_measurements:
            delta_x = shark.x - self.state.x
            delta_y = shark.y - self.state.y

            # added with Gaussian noise with 0 mean and standard deviation of 5
            z_shark_range = math.sqrt(delta_x**2 + delta_y**2) + np.random.normal(0, 5)
            # added with Gaussian noise with 0 mean and standard deviation of 0.5
            z_shark_bearing = angle_wrap(math.atan2(delta_y, delta_x) - self.state.theta + np.random.normal(0, 0.5))

            shark_state_list.append([shark.x, shark.y, z_shark_range, z_shark_bearing, shark.id])

        return shark_state_list


    def control_based_on_traj(self, auv_trajectory, new_trajectory, curr_time):
        """
        Using the auv_trajectory from the motion planner to determine the linear and angular
            velocity that the auv should have

        Parameters: 
            trajectory - a list of trajectory points, where each element is 
            new_trajectory - boolean, True, auv_trajectory is a new trajectory (reset counter)
                                      False, auv_trajectory is an old trajectory 
            curr_time - time in second, the current time in worldSim
            
        Return:
            linear velocity, angular velocity
                the velocities that the auv should have to head towards the waypoint
        """
        # control constant
        K_P = 0.5

        waypoint = self.find_waypoint_to_track(auv_trajectory, new_trajectory, curr_time)

        angle_to_traj_point = math.atan2(self.state.x - self.state.y, waypoint.x - self.state.x) 

        self.state.w = K_P * angle_wrap(angle_to_traj_point - self.state.theta) #proportional control
        
        return [self.state.v, self.state.w]


    def find_waypoint_to_track(self, auv_trajectory, new_trajectory, curr_time):
        """
        Return an MotionPlanState object representing the trajectory point TRAJ_LOOK_AHEAD_TIME sec ahead
        of current time

        Parameters: 
            trajectory - a list of trajectory points, where each element is 
            new_trajectory - boolean, True, auv_trajectory is a new trajectory (reset counter)
                                      False, auv_trajectory is an old trajectory 
            curr_time - time in second, the current time in worldSim
            
        Return:
            a MotionPlanState object, which represents the waypoint to track
        """
        # reset the trajectory index if it's a new trajectory
        if new_trajectory == True:
            self.curr_traj_pt_index = 0

        # only increment the index if it hasn't reached the end of the trajectory list
        while (self.curr_traj_pt_index < len(auv_trajectory)-1) and\
            (curr_time + self.TRAJ_LOOK_AHEAD_TIME) > auv_trajectory[self.curr_traj_pt_index].traj_time_stamp: 
            self.curr_traj_pt_index += 1

        return auv_trajectory[self.curr_traj_pt_index]

    
    def update_auv_state(self, auv_control_signals, delta_t):
        """
        Update the auv state (x, y, theta) based on the control signals

        Parameters:
            auv_control_signals - Python list, [linear_velocity, angular_velocity]
            delta - time step (sec)
        """
        v, w = auv_control_signals

        self.state.x = self.state.x + v * math.cos(self.state.theta) * delta_t
        self.state.y = self.state.y + v * math.sin(self.state.theta) * delta_t
        self.state.theta = angle_wrap(self.state.theta + w * delta_t)

        # add the new position for plotting
        self.x_list_plot.append(self.state.x)
        self.y_list_plot.append(self.state.y)
        self.z_list_plot.append(self.state.z)


    def create_auv_msg(self, shark_state_estimates, auv_trajectory):
        """
        Create communication message that contain information about this auv
            in the current iteration of control loop

        Parameter:
            shark_state_estimates - Python list, [x_estimated_shark, y_estimated_shark]
            auv_trajectory - Python list, list of trajectory points from the auv planner

        Return:
            dictionary, contains
                "auv_state": MotionPlanState object, which represents the current state of the auv
                "shark_est": Python list, [x_estimated_shark, y_estimated_shark]
                "auv_trajectory": Python list, list of trajectory points from the auv planner
            TODO: 
                add type of planner
                what the planner is planning for
        """
        msg = {
            "auv_state": self.state, 
            "shark_est": shark_state_estimates,
            "auv_trajectory": auv_trajectory
        }

        return msg
               


class MotionPlanState:
    def __init__(self, x, y, z=0, theta=0, v=0, w=0, traj_time_stamp=0, plan_time_stamp=0, size=0):
        """
        Initialilze a class to hold all the necessary information needed for motion planning

        Used in: 
            Auv.py
            WorldSim.py
            motionPlanners/A_star.py
            motionPlanners/RRT.py
        """
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

        self.v = v  # linear velocity
        self.w = w  # angular velocity

        self.traj_time_stamp = traj_time_stamp
        self.plan_time_stamp = plan_time_stamp

        self.size = size

        self.parent = None
        self.path = []
        self.length = 0
        self.cost = []

    def __repr__(self):
        return "MPS: [x=" + str(self.x) + ", y="  + str(self.y) + ", z=" + str(self.z) +\
                ", theta=" + str(self.theta)  + ", v=" + str(self.v) + ", w=" + str(self.w) + ", size=" + str(self.size) +\
                ", traj_time=" + str(self.traj_time_stamp) +  ", plan_time="+  str(self.plan_time_stamp) + "]"

    def __str__(self):
        return self.__repr__()


def main():
    env_info = {
        "obstacles": [MotionPlanState(757,243, size=2), MotionPlanState(763,226, size=5)],
        "boundary": [MotionPlanState(-500, -500), MotionPlanState(500,500)],
        "habitats": [MotionPlanState(63,23, size=5), MotionPlanState(12,45,size=7), 
                    MotionPlanState(51,36,size=5), MotionPlanState(45,82,size=5),
                    MotionPlanState(60,65,size=10), MotionPlanState(80,79,size=5),
                    MotionPlanState(85,25,size=6)]
    }

    # TODO: to be removed
    a = Auv(10, -10, 0, 0, 2, np.pi/4, ["astar"], env_info)


if __name__ == "__main__":
    main()