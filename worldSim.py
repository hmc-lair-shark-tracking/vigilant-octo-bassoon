"""
Is still going to require touch ups and integration. This is only to get a skeleton so far.
"""

#TODO Import the right files
import shark
import auv
import plot
import controller
import motion_plan_state
import cost
import catalina

# Define our constants
#TODO Change constants/variables to fit our needs

class WorldSim:

    def __init__(self, time_to_run, historical_data_set):
        self.MAX_TIME_TO_RUN = time_to_run
        self.auv_list = createAuvList() #What should I do for this?
        self.shark_list = createSharkList() #What should I do for this?
        self.historical_data_set = historical_data_set
        self.auv_comm_msgs = [] #What should I do for this?

    def runMainLoop(self):
        """
        Updates the sharks' states, updates the AUVs' states, and plots the world
        """
        for t in range(self.MAX_TIME_TO_RUN):
            self.shark_list.updateSharkStates(t)
            self.auv_comm_msgs = self.auv_list.updateAUVStates(t, self.shark_list, self.auv_comm_msgs)
            plot_world(self.shark_list, self.auv_list)

    def createAuvList(self, initial_states):
        """
        Creates a list of AUVs given their initial positions

        Parameters:
        intial_positions: list of the starting state of each AUV
        """
        auv_list = []
        for state in initial_states:
            auv = createAuv(initial_states) #Need to reference our AUV file
            auv_list.append(auv)
        
        return auv_list
        
    def createSharkList(self, initial_states):
        """
        Creates a list of Sharks given their initial positions

        Parameters:
        intial_positions: list of the starting state of each Shark
        """
        shark_list = []
        for state in initial_states:
            shark = createShark(initial_states) #Need to reference our AUV file
            shark_list.append(shark)
        
        return shark_list

    def updateSharkStates(self, t):
        """
        Takes in a timestamp and uses historical shark data or a probabilistic 
        motion model to estimate shark movement for each shark.

        Changes the states of sharks in shark_list
        """
        for shark in self.shark_list:
            if self.historical_data_set == None:
                shark.updateStateWithDataSet(t, self.historical_data_set)
            else:
                shark.updateStateWithSharkProbabilisticMotionModel(t)


    def updateAUVStates(self, t, shark_list, auv_comm_msgs):
        """
        Updates the AUVs' states based on the timestamp, the sharks' information, and the AUV comm messages

        Changes the states of AUVs in auv_list
        """
        new_comm_msgs = []
        for auv in self.auv_list:
            shark_measurements = simulateSharkMeasurements(auv.state, self.shark_list)
            new_msg = auv.runControlLoop(t, shark_measurements, self.auv_comm_msgs)
            new_comm_msgs.append(new_msg)
        self.auv_comm_msgs = new_comm_msgs
        


    def runControlLoop(self, t, shark_measurements, auv_comm_msgs):
        """
        Determines the control signals for an AUV(s) given timestamp, shark information, and AUV comm messages

        Returns a new comm message based on the updates to the AUV's state
        """
        shark_state_estimates = PF(shark_measurements, self.auv_comm_msgs)
        planner = MotionPlanner(auv_state, shark_state_estimates, self.auv_comm_msgs)
        auv_trajectory = planner.determine_traj()
        control_signals = Controller(auv.state, auv_trajectory)
        update_state(control_signals)
        return new_msg(auv.state, shark_measurements, auv_trajectory(s))

if __name__ == '__main__':
    MAX_TIME_TO_RUN = 30
    historical_data_set = []
    world_sim = WorldSim(MAX_TIME_TO_RUN, historical_data_set)
    world_sim.runMainLoop()
