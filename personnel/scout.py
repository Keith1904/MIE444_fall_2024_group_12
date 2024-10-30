import numpy as np
import random
import math
import pygame
import scipy.stats
import utilities

class Scout:
    '''This class performs the particle filter localization algorithm.'''
    
    def __init__(self, num_particles, maze, robot):
        # Define MCL Parameters
        self.num_particles = num_particles
        self.maze = maze
        self.particles = self.initialize_particles()
        self.robot = robot
    
    def initialize_particles(self):
        """Randomly generate the first batch of particles around the maze in valid locations."""
        particles = []
        for i in range(self.num_particles):
            while True:
                x = random.uniform(0, self.maze.size_x)
                y = random.uniform(0, self.maze.size_y)
                theta = random.uniform(0, 360)
                
                if self.is_valid_position(x, y):
                    particles.append(Particle(x, y, theta, 1 / self.num_particles))
                    break
        return particles
    
    def is_valid_position(self, x, y):
        """Checks if the location of a particle is valid."""
        
        # Convert x and y positions to indices in the walls attribute.
        x_cell = int(x / 12)
        y_cell = int(y / 12)
        
        # Check that the position is within the maze boundaries.
        if 0 <= x_cell < self.maze.size_x / 12 and 0 <= y_cell < self.maze.size_y / 12:
            return self.maze.orig_walls[y_cell][x_cell] != 0
        return False
    
    def predict(self):
        # Retrieve current and previous encoder readings
        m0_curr = self.robot.motor_encoders["m0"]["reading"]
        m0_prev = self.robot.motor_encoders["m0"]["previous_reading"]
        m1_curr = self.robot.motor_encoders["m1"]["reading"]
        m1_prev = self.robot.motor_encoders["m1"]["previous_reading"]
        
        # Calculate the change in each motor encoder
        delta_m0 = m0_curr - m0_prev
        delta_m1 = m1_curr - m1_prev
        
        # Compute average displacement and change in orientation
        d = (delta_m0 + delta_m1) / 2
        if abs(d) < 0.001:
            d = 0
        
        delta_theta = (delta_m1 - delta_m0) / self.robot.wheel_distance
        if abs(delta_theta) < 1e-6:
            delta_theta = 0

        # Define the standard deviations for noise in distance and angle
        distance_noise_std = 0.1  # Adjust based on your observations
        angle_noise_std = 0.5     # Adjust based on your observations

        for particle in self.particles:
            # Add Gaussian noise to the distance and angle
            d_noisy = d + np.random.normal(0, distance_noise_std)
            delta_theta_noisy = delta_theta + math.radians(np.random.normal(0, angle_noise_std))
            
            # Update the bearing in degrees
            theta_new = particle.theta + math.degrees(delta_theta_noisy)
            # Normalize theta to stay within 0 to 360 degrees
            theta_new = theta_new % 360
            
            if delta_theta_noisy == 0:
                # Moving in a straight line
                x_new = particle.x + d_noisy * math.cos(math.radians(particle.theta))
                y_new = particle.y - d_noisy * math.sin(math.radians(particle.theta))  # Subtract to account for downward y-axis
            else:
                # Moving in an arc
                R = d_noisy / delta_theta_noisy  # Use noisy delta_theta for calculation
                x_new = particle.x - R * math.sin(math.radians(particle.theta)) + R * math.sin(math.radians(theta_new))
                y_new = particle.y - (R * math.cos(math.radians(particle.theta)) - R * math.cos(math.radians(theta_new)))  # Account for downward y-axis
            
            # Update the particle's position
            particle.update_position(x=x_new, y=y_new, theta=theta_new)
            
    def update_weights(self, maze, robot, R):
        """
        Updates weights based on the similarity between simulated and actual sensor readings.
        """
        # Sensor IDs (u0: front, u1: right, u2: back, u3: left)
        sensor_ids = ["u0", "u1", "u2", "u3"]

        for sensor_id in sensor_ids:  # Iterate over each sensor (u0, u1, u2, u3)
            # Retrieve the actual reading for the current sensor from the robot
            actual_reading = robot.distance_sensors[sensor_id]["reading"]

            for particle in self.particles:
                # Simulate the reading for the current particle and sensor
                simulated_distance = particle.simulate_ultrasonic_sensor(maze, robot, sensor_id)
                
                # Calculate the likelihood of the actual reading given the simulated distance
                likelihood = scipy.stats.norm(simulated_distance, R).pdf(actual_reading)
                # Update the particle's weight
                particle.weight *= likelihood
                particle.weight += 1e-300

        # Normalize the weights to sum to 1
        total_weight = sum(particle.weight for particle in self.particles)
        for particle in self.particles:
            particle.weight = particle.weight / total_weight if total_weight != 0 else 0
                 
    def systematic_resample(self):
        """
        Perform systematic resampling based on the weights of the particles.
        
        Parameters:
            particles (list): List of particle objects, each having an attribute 'weight'.

        Returns:
            indexes (ndarray): Array of indices for resampling particles.
        """
        N = len(self.particles)
        weights = np.array([particle.weight for particle in self.particles])
        positions = (np.arange(N) + np.random.random()) / N
        indexes = np.zeros(N, dtype=int)
        cumulative_sum = np.cumsum(weights)

        i, j = 0, 0
        while i < N:
            if positions[i] < cumulative_sum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1

        return indexes

    def resample_from_index(self, indexes):
        """
        Resample the particles using the provided indices and normalize weights.
        
        Parameters:
            particles (list): List of particle objects with 'x', 'y', 'theta', and 'weight' attributes.
            indexes (ndarray): Array of indices indicating which particles to keep.

        Returns:
            None: The particles list is modified in place.
        """
        # Create a copy of the current particles based on the indexes
        new_particles = [self.particles[i] for i in indexes]

        # Replace old particles with the resampled particles
        for i, new_particle in enumerate(new_particles):
            self.particles[i].x = new_particle.x
            self.particles[i].y = new_particle.y
            self.particles[i].theta = new_particle.theta  # Update the theta attribute as well
            self.particles[i].weight = 1.0 / len(self.particles)  # Reset weight to be equal

        # Normalize the weights to sum to 1
        total_weight = sum(particle.weight for particle in self.particles)
        for particle in self.particles:
            particle.weight /= total_weight if total_weight != 0 else 1.0
    
    def neff(self):
        weights = np.array([particle.weight for particle in self.particles])
        #print(weights)
        return 1. / np.sum(np.square(weights))
    
class Particle:
    '''Defines the attributes of a particle including it's position and weight.'''
    
    def __init__(self, x, y, theta, weight=1.0):
        self.x = x                  # Particle's x position
        self.y = y                  # Particle's y position
        self.theta = theta  # Particle's bearing direction
        self.weight = weight        # Particle's weight (probability)
    
    def update_weight(self, new_weight):
        """Update the weight of the particle."""
        self.weight = new_weight
    
    def update_position(self, x = 0, y = 0, theta = 0):
        self.x = x
        self.y = y
        self.theta = theta
        
    def simulate_ultrasonic_sensor(self, maze, robot, sensor_id):
        sensor_angle = self.theta - robot.distance_sensors[sensor_id]["rotation"]
        length = pygame.math.Vector2(100,0)
        sensor_x = self.x + robot.distance_sensors[sensor_id]["y"] * math.cos(math.radians(self.theta)) + robot.distance_sensors[sensor_id]["x"] * math.sin(math.radians(self.theta)) 
        sensor_y = self.y + robot.distance_sensors[sensor_id]["x"] * math.cos(math.radians(self.theta)) - robot.distance_sensors[sensor_id]["y"] * math.sin(math.radians(self.theta))
        beam_end = pygame.math.Vector2.rotate(length, -sensor_angle) + [sensor_x, sensor_y]
        beam = [pygame.math.Vector2(sensor_x, sensor_y), beam_end]
        
        #print(f"beam before: {beam}")
        walls_to_check = maze.reduced_walls
        squared_distance = 100^2
        for wall in walls_to_check:
            collision_points = utilities.collision(beam, wall)
            if not collision_points:
                pass
            else:
                beam[1], squared_distance = utilities.closest_fast([sensor_x, sensor_y], collision_points)
        return math.sqrt(squared_distance)