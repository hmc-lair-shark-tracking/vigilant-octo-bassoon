from RRT import RRT
from A_star import astar

class MotionPlanner:
    def __init__(self, planner_key_list, env_info):

        """
        Initialize a MotionPlanner class given the type of planner employed and environmental information

        Parameters:
            planner_key_list - Python list, indicate which planner(s) to use in this auv
            env_info - Python dictionary, contain information neccessary for the planner
                (e.g. boundary, obstacles, habitats)
        """
        if planner_key_list[0] == "astar":
            self.motion_planner = astar(env_info)
        elif planner_key_list[0] == "rrt":
            self.motion_planner == RRT(env_info)
        self.env_info = env_info

    def plan_trajectory(self, auv_comm_msgs):
        
        """
        Return the trajectory planned by the specified planner

        Parameter:
            auv_comm_msgs - Python list of dictionaries, representing the information sent by other auvs
        """
        return self.motion_planner.plan_trajectory(auv_comm_msgs)


