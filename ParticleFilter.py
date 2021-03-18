import math
import numpy as np
import random
from copy import copy, deepcopy

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

def velocity_wrap(velocity):
    """
    Return:
        transform a velocity to under 5 m/s
    """
    if velocity <= 5:
        return velocity  
    elif velocity > 5: 
        velocity += -5
        return velocity_wrap(velocity)
    

class ParticleFilter:
    def __init__(self):
        """
        Initialize a ParticleFilter class given the initial shark position

        Paramters: 
            x_shark - x position of a shark
            y_shark - y position of a shark
            number_of_particles - optional (default set to 150), the total number of particles in the Particle_Filter class
            initial_particle_range - the side length of square that the random particles are in
        """
        self.particles_list = []


    def create_particles(init_shark_state, particles_num=1000, init_particle_range=150):
        """
        Initialize Particles and store them into particle_list

        Parameters:
            init_shark_state - Motion Plan State, contains the shark's postion
            number_of_particles - optional (default set to 1000), the total number of particles in the Particle_Filter class
        """
        for _ in range(particles_num):
            self.particles_list.append(Particle(init_shark_state, particles_num, init_particle_range))


    def estimate_shark_state(auv_state, shark_state_list, delta_t):
        """
        Estimate the shark's state based on current measurement
            (Update the particles first, then make estimation)
        Called by the AUV class

        Parameters: 
            auv_state - a Motion_planning_state object, mainly using the auv's theta
            shark_state_list - Python list of lists, each element is the measurement for a shark, 
                each element's format: [x_shark, y_shark, z_shark_range, z_shark_bearing, shark_id]
            delta_t - the amount of time the particles are "moving" 
                (generally set to .1, but it should match how long an iteration of the WorldSim whatever the "time.sleep" is set to in the main loop

        Return:
            estimated_pos - a Python list: [x_estimated_shark, y_estimated_shark]
            estimated_error - mean square error of the estimation
        """
        # randomly advance the particles position
        for particle in self.particles_list:
            particle.move_particle(delta_t)

        self.update_all_particle(auv_state, shark_state_list)

        estimated_pos = self.particle_mean()

        estimation_error = self.particle_mean_error(estimated_pos, shark_state_list)

        return estimated_pos, estimated_error


    def update_all_particles(auv_state, shark_state_list):
        """
        Update all the particles following these state:
            1. based on current shark measurement, calculate particle's weight
            2. normalize all particles' weights
            3. create a new sets of particles based on the weights

        Parameters: 
            auv_state - a Motion_planning_state object, mainly using the auv's theta
            shark_state_list - Python list of lists, each element is the measurement for a shark, 
                each element's format: [x_shark, y_shark, z_shark_range, z_shark_bearing, shark_id]
        """
        # list of lists, one list per shark_state
        weights_list_per_shark = []

        # calculate particles' weights based on individual shark's measurement
        for shark_state in shark_state_list:
            curr_shark_weights_list = []
            for particle in self.particles_list:
                curr_shark_weights_list.append(particle.weight(auv_state, shark_state))
            weights_list_per_shark.append(curr_shark_weights_list)

        normalized_weights_list = self.normalized_weights(weights_list_per_shark)

        # actually update individual particle's weight
        for i in range(len(self.particles_list)):
            self.particles_list[i].weight_p = normalized_weights_list[i]
        
        # self.particles_list gets updated (favor particles with higher weights)
        self.correct_particles()


    def normalize_weights(weights_list_per_shark):
        """
        Normalize the weights following these steps:
            1. individually normalize each weight list by its max denominator
            2. adds all the weight list together then normalize it

        Parameter:
            weights_list_per_shark - Python list, list of lists, 
                each element represent the list of particle weights 
                based on a specific shark measurement

        Return:
            Python list, a 1 by num_of_particles list, which is the normalized weight
        """
        # normalize individual list based on highest value
        individually_norm_weights_list = []
        for weights_list in weights_list_per_shark:
            denominator = max(weights_list)
            # elementally divide each weight with the denominator
            individually_norm_weights_list.append(np.array(weights_list) / denominator)
        
        # a num_of_shark_measurements by num_of_particles matrix
        #   basically len(weights_list_per_shark) by len(weights_list) matrix
        individually_norm_weights_list = np.array(individually_norm_weights_list)
        # add all the rows together
        #   a 1 by num_of_particles vector
        normalized_weights_list = individually_norm_weights_list.sum(axis=0)
        # normalize the added weights by the highest value
        normalized_weights_list = normalized_weights_list / max(normalized_weights_list)

        # convert the numpy array back to python list
        return normalized_weights_list.tolist()


    def correct_particles(self):
        """
        Based on the weights of the current particles, create a new set of particles
            where higher-weighted particles would be favored when creating the new set
            (total number of particles stay the same)

        Set the new set of particles to self.particles_list
        """
        # number of copies based on weights
        COPY_NUM_LESS_THAN_02 = 1  # weight < 0.2
        COPY_NUM_LESS_THAN_04 = 2  # weight < 0.4
        COPY_NUM_LESS_THAN_06 = 3  # weight < 0.6
        COPY_NUM_LESS_THAN_08 = 4  # weight < 0.8
        COPY_NUM_LESS_THAN_1 = 5  # weight <= 1.0

        new_particles_list = []
        particles_num = len(self.particles_list)

        for particle in self.particles_list:
            if particle.weight_p < 0.2:
                for _ in range(COPY_NUM_LESS_THAN_02):
                    new_particles_list.append(deepcopy(particle))
            elif particle.weight_p < 0.4:
                for _ in range(COPY_NUM_LESS_THAN_04):
                    new_particles_list.append(deepcopy(particle))
            elif particle.weight_p < 0.6:
                for _ in range(COPY_NUM_LESS_THAN_06):
                    new_particles_list.append(deepcopy(particle))
            elif particle.weight_p < 0.8:
                for _ in range(COPY_NUM_LESS_THAN_08):
                    new_particles_list.append(deepcopy(particle))
            elif particle.weight_p <= 1:
                for _ in range(COPY_NUM_LESS_THAN_1):
                    new_particles_list.append(deepcopy(particle))
        
        # randomly pick particle_nums of new particles from the new_particles_list
        self.particles_list = random.choice(new_particles_list, k = particle_nums)


    def particle_mean(self):
        """
        Based on the current particles, estimate the shark position
            (which is the mean of all the particles' position)

        Return:
            Python list, [x_estimated_shark, y_estimated_shark]
        """
        particle_x_sum = 0
        particle_y_sum = 0
        particle_nums = float(len(self.particles_list))

        for particle in self.particles_list:
            particle_x_sum += particle.x_p
            particle_y_sum += particle.y_p

        return [float(particle_x_sum) / particle_nums, float(particle_y_sum) / particle_nums]


    def particle_mean_error(self, estimated_pos, shark_state_list):
        """
        Parameter: 
            estimated_pos - Python list, [x_estimated_shark, y_estimated_shark]
            shark_state_list - Python list of lists, each element is the measurement for a shark, 
                each element's format: [x_shark, y_shark, z_shark_range, z_shark_bearing, shark_id]
        
        Return: 
            mean square error of the estimated shark position
        """
        x_estimated_shark, y_estimated_shark = estimated_pos

        # find the "actual" shark position first
        x_actual_shark = 0
        y_actual_shark = 0
        for shark_state in shark_state_list:
            x_shark, y_shark, _, _, _ = shark_state
            x_actual_shark += x_shark
            y_actual_shark += y_shark
        x_actual_shark = mean(x_actual_shark)
        y_actual_shark = mean(y_actual_shark)

        return math.sqrt((x_estimated_shark - x_actual_shark)**2 + (y_estimated_shark - y_actual_shark)**2)

    
    def all_particles_coordinates(self):
        """
        Debugging function

        Return:
            Python list of list, represent all the position and weights of the particles
                each element has format: [particle_x, particle_y, particle_weight]
        """
        coordinates_list = []

        for particle in self.particles_list:
            coordinates_list.append([particle.x_p, particle.y_p, particlel.weight_p])
        
        return coordinates_list

    
    def __repr__(self):
        """
        Define what ParticleFilter class's print looks like
        """
        print("Particle Filter (each element: [particle_x, particle_y, particle_weight])")
        print(self.all_particles_coordinates())


class Particle:
    def __init__(self, init_shark_state, particle_nums, init_particle_range):
        """
        Initialize a Particle given the initial shark position

        Paramter: 
            x_shark - x position of a shark
            y_shark - y position of a shark
            particle_nums - the total number of particles in the ParticleFilter class
            init_particle_range - the side length of square that the random particles are in
        """
        # particle has 5 properties: x, y, velocity, theta, weight (starts at 1/N)
        self.x_p = init_shark_state.x + np.random.uniform(-init_particle_range, init_particle_range)
        self.y_p = init_shark_state.y + random.uniform(-init_particle_range, init_particle_range)
        self.v_p = random.uniform(0, 5)
        self.theta_p = random.uniform(-math.pi, math.pi)
        self.weight_p = 1/particles_num


    def move_particle(self, delta_t):
        """
        updates the particle's location with random v and theta

        Parameter:
            delta_t - the amount of time the particles are "moving" 
                (generally set to .1, but it should match how long an iteration of the WorldSim whatever the "time.sleep" is set to in the main loop
        """
        # random_v and random_theta are values to be added to the velocity and theta for randomization
        RANDOM_VELOCITY = 5
        RANDOM_THETA = math.pi/2

        # change velocity & pass through velocity_wrap
        self.v_p += random.uniform(0, RANDOM_VELOCITY)
        self.v_p = velocity_wrap(self.v_p)

        # change theta & pass through angle_wrap
        self.theta_p += random.uniform(-RANDOM_THETA, RANDOM_THETA)
        self.theta_p = angle_wrap(self.theta_p)

        # update the x, y position based on the new v and theta
        self.x_p += self.v_p * math.cos(self.theta_p) * dt
        self.y_p += self.v_p * math.sin(self.theta_p) * dt


    def calc_particle_alpha(self, x_shark, y_shark, theta_auv):
        """
        Parameter:
            x_auv - x position of the shark
            y_auv - y position of the auv
            TODO: from the old auv.py get_all_sharks_sensor_measurements function
                these are shark measurement, but they are labeled as x_auv and y_auv in the
                old particleFilter.py
            theta_auv - theta (in radians) of the auv
        
        Return:
            the alpha value of a particle based on the auv position
        """
        return angle_wrap(math.atan2((-y_shark + self.y_p), (self.x_p + -x_shark)) - theta_auv)


    def calc_particle_range(self, x_shark, y_shark):
        """
        Parameter:
            x_shark - x position of the shark
            y_shark - y position of the shark
            TODO: from the old auv.py get_all_sharks_sensor_measurements function
                these are shark measurement, but they are labeled as x_auv and y_auv in the
                old particleFilter.py
        Return:
            the range from the particle to the auv
        """
        return math.sqrt((y_shark - self.y_p)**2 + (x_shark - self.x_p)**2)


    def calc_weight(self, auv_state, shark_state):
        """
        Calculate a particle's weight according to alpha, then according to range
        The two results are multiplied together to get the final weight

        Parameter:
            auv_state - a Motion_planning_state object, mainly using the auv's theta
            shark_state - a python list representing a shark's measurement
                of format: [x_shark, y_shark, z_shark_range, z_shark_bearing, shark_id]

        Return:
            the calculated particle weight

        Warning:
            does not actually update the particle's weight
        """
        theta_auv = auv_state.theta
        
        x_shark, y_shark, z_shark_range, _, _ = shark_state

        # Constants used in the calculation
        SIGMA_ALPHA = 0.5   # alpha weight
        SIGMA_RANGE = 100   # range weight
        CONSTANT = 1.2533141375
        MINIMUM_WEIGHT = .001

        particle_alpha = self.calc_particle_alpha(x_shark, y_shark, theta_auv)
        particle_range = self.calc_particle_range(x_shark, y_shark)

        if particle_alpha > 0:
            weight_p = 0.001 + (1.0/(constant) * (math.e**(((-((angle_wrap(float(particle_alpha) - z_shark_range)**2))))/(0.5))))
        elif particle_alpha == 0:
            weight_p = 0.001 + (1.0/(constant) * (math.e**(((-((angle_wrap(float(particle_alpha) - z_shark_range)**2))))/(0.5))))
        else:
            weight_p = 0.001 + (1.0/(constant) * (math.e**(((-((angle_wrap(float(particle_alpha) - z_shark_range)**2))))/(0.5))))

        # multiply weights
        weight_p *= MINIMUM_WEIGHT + (1/(SIGMA_RANGE * constant)* (math.e**(((-((particle_range - z_shark_range)**2)))/(20000))))

        return weight_p


    def __repr__(self):
        """
        Define what Particle class's print looks like
        """
        print("Particle(x={self.x_p}, y={self.y_p}, weight={self.weight_p})")
