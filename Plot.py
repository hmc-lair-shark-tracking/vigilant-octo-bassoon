import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.widgets import Button, CheckButtons

import numpy as np

class Plot2D:
    def __init__(self, num_of_auv=1, shark_id_list=[]):
        # list of pre-defined colors, 
        # so we can draw sharks with different colors
        self.colors = ['b', 'g', 'c', 'm', 'y', 'k']

        # initialize the 2d graph
        self.fig = plt.figure(figsize = [13, 10])
        self.ax = self.fig.add_subplot(111)

        # create a dictionary for checkbox for each type of planned trajectory
        # key - the planner's name: "A *", "RRT"
        # value - three-element list
        #   1. boolean(represent wheter the label is added to the legend)
        #   2. the CheckButtons object
        #   3. color of the plot
        self.traj_checkbox_dict = {}
        
        # initialize the A * button
        self.traj_checkbox_dict["A *"] = [False, CheckButtons(plt.axes([0.1, 0.90, 0.15, 0.05]), ["A* Trajectory"]), '#9933ff']
        # when the A* checkbox is checked, it should call self.enable_traj_plot

        # initialize the RRT button
        self.traj_checkbox_dict["RRT"] = [False, CheckButtons(plt.axes([0.25, 0.90, 0.15, 0.05]),["RRT Trajectory"]), '#043d10']
        # when the RRT checkbox is checked, it should call self.enable_traj_plot

        # initialize the particle buttle
        self.particle_checkbox = CheckButtons(plt.axes([0.40, 0.90, 0.15, 0.05]),["Display Particles"])
        self.display_particles = False
        self.particle_checkbox.on_clicked(self.particle_checkbox_clicked)

        # an list of the labels that will appear in the legend
        self.labels = []

        # initialize the labels for the auv
        self.load_auv_labels(num_of_auv)
        self.load_shark_labels(shark_id_list)

    
    def plot_world(self, auv_pos_list=[], shark_pos_list=[], trajectory_list=[], particle_list=[], obstacles_list=[]):
        """
        Plot the current world in 2d

        Parameters:
            auv_pos_list - optional, list of lists, each element contains the x and y position of a auv
                each element has the format: [auv_x_list, auv_y_list]
                    auv_x_list, auv_y_list: a list of numbers
            shark_pos_list - optional, list of lists, each element contains the x and y position of a shark
                each element has the format: [shark_x_list, shark_y_list, shark_id]
                    shark_x_list, shark_y_list: a list of numbers
                    shark_id: a number
            trajectory_list - optional, list of lists, each element contains the trajectory based on a specific planner
                each element has the format: [planner_name, trajectory_points_list]
                    planner_name should be either 'RRT' or 'A *'
                    trajectory_points_list: a list of MotionPlanState object
            particle_list - optional, list of lists, each element represent a particle's information
                each element has the format: [particle_x, particle_y, particle_weight],
                    which are all numbers
            obstacle_list - optional, list of MotionPlanState object
        """
        for i in range(len(auv_pos_list)):
            auv_x_list, auv_y_list = auv_pos_list[i]

            self.plot_entity(auv_x_list, auv_y_list, label = "auv #" + str(i))

        for i in range(len(shark_pos_list)):
            shark_x_list, shark_y_list, shark_id = shark_pos_list[i]

            self.plot_entity(shark_x_list, shark_y_list, label = "shark #" + str(shark_id), color = self.colors[i % len(self.colors)])

        for trajectory in trajectory_list:
            planner_name, trajectory_points_list = trajectory

            self.plot_planned_traj(planner_name, trajectory_points_list)

        if particle_list != []:
            self.plot_particles(particle_list)

        if obstacles_list != []:
            self.plot_obstacles(obstacles_list)

        self.ax.legend(self.labels)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        plt.draw()

        # pause so the plot can be updated
        plt.pause(0.0001)

        self.ax.clear()


    def plot_entity(self, x_pos_list, y_pos_list, label = 'auv', color = 'red', marker = ','):
        """
        Plot the entity's trajectory in 2D graph

        Parameters:
            x_pos_list - list of floats indicating the entity's past x-position
            y_pos_list - list of floats indicating the entity's past y-position
            label - optional (default: auv)
                string specifying what would the line be labeled as in the legend
            color - optional (default: red)
                the color of the trajectory
            marker - optional (default: ',' which just plots a line without showing intermediate point)
        """
        self.ax.plot(x_pos_list, y_pos_list,
            marker = marker, linestyle = '-', color = color, label = label)


    def enable_traj_plot(self, event):
        """
        Handles when a check box is hit

        Parameter: 
            event - a string, matches with the name of the label when the checkButton is created
        """
        if (event == "A* Trajectory"):
            # self.traj_checkbox_dict["A *"][0] returns whether the label has been added to 
            #   self.labels aka the legend
            # we only want one copy of the label to be in self.labels
            if not self.traj_checkbox_dict["A *"][0]:
                self.labels += ["A *"]
                self.traj_checkbox_dict["A *"][0] = True
        elif (event == "RRT Trajectory"):
            if not self.traj_checkbox_dict["RRT"][0]:
                self.labels += ["RRT"]
                self.traj_checkbox_dict["RRT"][0] = True      

    
    def plot_planned_traj(self, planner_name, trajectory_list):
        """
        Plot the planned trajectory specified by the planner name

        Parameters:
            planner_name - string, either "A *" or "RRT"
            trajectory_list - an list of Motion_plan_state objects
        """
        # get the checkbox object
        checkbox = self.traj_checkbox_dict[planner_name][1]
        # boolean, true if the checkbox is checked
        checked = checkbox.get_status()[0]
        # get the color of the trajectory plot (a string representing color in hex)
        color = self.traj_checkbox_dict[planner_name][2]
        
        if checked:
            # self.traj_checkbox_dict["A *"][0] returns whether the label has been added to 
            #   self.labels aka the legend
            # we only want one copy of the label to be in self.labels 
            if not self.traj_checkbox_dict[planner_name][0]:
                self.labels += [planner_name]
                self.traj_checkbox_dict[planner_name][0] = True
            
            self.ax.plot([mps.x for mps in trajectory_list],  [mps.y for mps in trajectory_list], marker = ',', color = color, label = planner_name)
        else:
            # if the checkbox if not checked
            # self.traj_checkbox_dict[planner_name][0] represents whether the label is added to
            #   self.label list
            # we only want to remove the label once
            if self.traj_checkbox_dict[planner_name][0]:
                self.labels.remove(planner_name)
                self.traj_checkbox_dict[planner_name][0] = False


    def particle_checkbox_clicked(self, event):
        """
        on_clicked handler function for particle checkbox

        toggle the display_particles variable (bool)
        """
        self.display_particles = not self.display_particles


    def plot_particles(self, particle_coordinates):
        """
        Plot the particles if the the particle checkbox is checked

        Parameter:
            particle_list - list of lists, where each element has the format:
                [x_p, y_p, weight_p]
        """
        if self.display_particles:
            particle_x_list = []
            particle_y_list = []
            particle_color_list = []
            # create two lists for plotting x and y positions
            for particle in particle_coordinates:
                x_p, y_p, weight_p = particle
                particle_x_list.append(x_p)
                particle_y_list.append(y_p)
                # particle[4] specify the weight of the points
                # the color of particles based on high weight to low weight:
                #   red -> orange -> purple -> blue
                if weight_p > 0.75 and weight_p <= 1.0:
                    # red
                    particle_color_list.append('#e31263')
                elif weight_p > 0.5 and weight_p <= 0.75:
                    # orange
                    particle_color_list.append('#912951')
                elif weight_p > 0.25 and weight_p <= 0.5:
                    # purple
                    particle_color_list.append('#7a5b67')
                else:
                    # blue
                    particle_color_list.append('#786b70')

            self.ax.scatter(particle_x_list, particle_y_list, marker = 'o', color = particle_color_list)

    
    def plot_obstacles(self, obstacles_list):
        """
        Plot the obstacles

        Parameter:
            obstacle_list - list of MotionPlanState (using x and y)
        """
        for obstacle in obstacles_list:
            self.ax.add_patch(plt.Circle((obs.x, obs.y), obs.size, color = '#000000', fill = False))

     
    def load_auv_labels(self, num_of_auv):
        """
        Add all the auvs to the legend
        
        Parameter:
            num_of_auv - number of auvs in the worldSim
        """
        for i in range(1, num_of_auv+1):
             # create legend with auvs
            self.labels += ["auv #" + str(i)]

    def load_shark_labels(self, shark_id_list):
        """
        All all the sharks (and their corresponding id) to the legend

        Parameter:
            shark_id_list - list of numbers
        """
        for shark_id in shark_id_list:
            self.labels += ["shark #" + str(shark_id)]
    

def main():
    # TODO: to be removed
    plot = Plot2D(shark_id_list=[5,7])

    auv_pos_list = [[[0], [0]]]


    shark_pos_list = [
        [[3], [3], 5],
        [[-1], [-1], 7]
    ]

    trajectory_list = [
        ["RRT", [MotionPlanState(x + 1, x-1) for x in range(4, 20)]],
        ["A *", [MotionPlanState(x - 2, x-2) for x in range(-4, 14)]]
    ]

    particle_list = []
    obstacles_list = []

    for _ in range(1000):
        plot.plot_world(auv_pos_list, shark_pos_list, trajectory_list, particle_list, obstacles_list)

        auv_x, auv_y = auv_pos_list[0]

        auv_x += [auv_x[-1] + 2 * np.random.rand()]
        auv_y += [auv_y[-1] + 2 * np.random.rand()]

        auv_pos_list = [[auv_x, auv_y]]

        new_shark_pos_list = []
        for shark in shark_pos_list:
            shark_x, shark_y, shark_id = shark

            shark_x += [shark_x[-1] + 1 * np.random.rand()]
            shark_y += [shark_y[-1] + 1 * np.random.rand()]
            
            new_shark_pos_list += [[shark_x, shark_y, shark_id]]

        shark_pos_list = new_shark_pos_list


class MotionPlanState:
    # TODO: currently using for debugging, remove 
    # class for motion planning
    def __init__(self, x, y, z=0, theta=0, v=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.v = v  # linear velocity
        self.w = w  # angular velocity


if __name__ == "__main__":
    main()


        