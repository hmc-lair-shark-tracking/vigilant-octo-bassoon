from RRT import RRT
from A_star import astar

class MotionPlanner:
    def __init__(self, planner_key_list, env_info, auv_comm_msgs):

        """
        Initialize a MotionPlanner class given the type of planner employed and environmental information

        Parameters:
            planner_key_list - Python list, indicate which planner(s) to use in this auv
            env_info - Python dictionary, contain information neccessary for the planner
                (e.g. boundary, obstacles, habitats)
        """
        self.planner_type = planner_key_list[0]
        self.env_info = env_info
        self.auv_comm_msgs = auv_comm_msgs
        self.auv_trajectory = []
    
    def astar_plan(self):
        A_star_planner = astar(self.env_info, self.auv_comm_msgs)
        auv_trajectory = A_star_planner.astar()
        return auv_trajectory

    def rrt_plan(self):
        rrt_planner = RRT(self.env_info, self.auv_comm_msgs)
        auv_trajectory = rrt_planner.rrt()
        return auv_trajectory
    
    def MotionPlanner(self):
        if self.planner_type == "astar":
            self.auv_trajectory = self.astar_plan()
        elif self.planner_type == "rrt":
            self.auv_trajectory = self.rrt_plan()


