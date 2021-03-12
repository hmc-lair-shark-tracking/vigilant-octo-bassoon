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


"""
TODO Create this as an object that initialize/create AUVS, create sharks, ...
"""


MAX_TIME_TO_RUN = 30
auv_list = []
shark_list = []
historical_data_set = []

def RunMainLoop():
    """
    Updates the sharks' states, updates the AUVs' states, and plots the world
    """
    for t in range(MAX_TIME_TO_RUN):
        shark_list.UpdateSharkStates(t)
        auv_comm_msgs = auv_list.UpdateAUVStates(t, shark_list, auv_comm_msgs)
        plot_world(shark_list, auv_list)


def UpdateSharkStates(t):
    """
    Takes in a timestamp and uses historical shark data or a probabilistic 
    motion model to estimate shark movement for each shark.

    Changes the states of sharks in shark_list
    """
    for shark in shark_list:
        if historical_data_set == nil:
            shark.UpdateStateWithDataSet(t, historical_data_set)
        else:
            shark.UpdateStateWithSharkProbabilisticMotionModel(t)


def UpdateAUVStates(t, shark_list, auv_comm_msgs):
    """
    Updates the AUVs' states based on the timestamp, the sharks' information, and the AUV comm messages

    Changes the states of AUVs in auv_list
    """
    new_comm_msgs = []
    for auv in auv_list:
        shark_measurements = SimulateSharkMeasurements(auv.state, shark_list)
        new_msg = auv.RunControlLoop(t, shark_measurements, auv_comm_msgs)
        new_comm_msgs.append(new_msg)


def RunControlLoop(t, shark_measurements, auv_comm_msgs):
    """
    Determines the control signals for an AUV(s) given timestamp, shark information, and AUV comm messages

    Returns a new comm message based on the updates to the AUV's state
    """
    shark_state_estimates = PF(shark_measurements, auv_comm_msgs)
    planner = MotionPlanner(auv_state, shark_state_estimates, auv_comm_msgs)
    auv_trajectory = planner.determine_traj()
    control_signals = Controller(auv_state, auv_trajectory)
    update_state(control_signals)
    return new_msg(auv_state, shark_measurements, auv_trajectory(s))


if __name__ == '__main__':
    main()