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
            
    def update_weights(self, maze, robot, sigma=0.05, epsilon=1e-6):
        """
        Update the weight of each particle based on the actual sensor readings.

        :param particles: List of particle objects, each with attributes (x, y, theta, weight).
        :param actual_readings: Dictionary of actual sensor readings keyed by sensor ID (e.g., 'u0', 'u1', etc.).
        :param sigma: Standard deviation of sensor noise.
        :param epsilon: A small value to prevent division by zero during normalization.
        """
        total_weight = 0.0

        sensor_ids = ["u0", "u1", "u2", "u3"]
        for particle in self.particles:
            # Initialize the weight of this particle
            particle.weight = 1.0

            # Loop over each sensor to compare readings
            for sensor_id in sensor_ids:
                # Simulate the sensor reading for this particle
                simulated_reading = particle.simulate_ultrasonic_sensor(maze, robot, sensor_id)
                
                # Get the actual reading for this sensor
                actual_reading = robot.distance_sensors[sensor_id]["reading"]
                
                # Adjust sigma based on the reading (e.g., linearly proportional to distance)
                adaptive_sigma = sigma * actual_reading  # or try sqrt(actual_reading)
                
                # Compute the likelihood using a Gaussian function with adaptive_sigma
                weight = self.gaussian(actual_reading, adaptive_sigma, simulated_reading)
                
                # Update the particle's weight
                particle.weight *= weight

            # Accumulate total weight for normalization later
            total_weight += particle.weight

        # Check if total_weight is zero and normalize
        if total_weight < epsilon:
            print("Warning: Total weight is too close to zero, skipping normalization.")
            # Optionally reinitialize particles if total weight is zero to avoid collapse
            for particle in self.particles:
                particle.weight = 1.0 / len(self.particles)
        else:
            # Normalize weights
            for particle in self.particles:
                particle.weight /= total_weight

    def gaussian(self, mean, sigma, x):
        """
        Gaussian probability density function.
        
        :param mean: Mean of the Gaussian distribution (actual sensor reading).
        :param sigma: Standard deviation of the Gaussian (sensor noise).
        :param x: Simulated sensor reading for the particle.
        :return: Probability of the observed value given the mean and sigma.
        """
        return (1.0 / (sigma * math.sqrt(2.0 * math.pi))) * math.exp(-0.5 * ((x - mean) / sigma) ** 2)
                 
    def resample(self):
        """
        Resample particles with replacement based on their weights.

        :return: New list of resampled particles.
        """
        new_particles = []
        N = len(self.particles)

        # Create an array of cumulative weights
        cumulative_weights = [0.0] * N
        cumulative_weights[0] = self.particles[0].weight
        for i in range(1, N):
            cumulative_weights[i] = cumulative_weights[i-1] + self.particles[i].weight

        # Generate a random starting point
        r = random.uniform(0, 1 / N)
        
        # Perform systematic resampling
        index = 0
        for i in range(N):
            threshold = r + i / N
            while threshold > cumulative_weights[index]:
                index += 1
            # Create a copy of the selected particle and reset its weight
            selected_particle = self.copy_particle(self.particles[index])
            selected_particle.weight = 1.0 / N
            new_particles.append(selected_particle)

        self.particles = new_particles  # Replace old particles with new resampled ones

    def copy_particle(self, particle):
        """
        Create a deep copy of a particle.
        Adjust this function based on the actual attributes of your particle.
        """
        return Particle(particle.x, particle.y, particle.theta, particle.weight)  # Modify as per your Particle class

    def compute_neff(self):
        """
        Calculate the effective sample size (Neff) of the particles based on their weights.

        :return: The effective sample size (Neff).
        """
        sum_of_squares = sum(particle.weight ** 2 for particle in self.particles)
        if sum_of_squares == 0:  # Avoid division by zero
            return 0
        return 1.0 / sum_of_squares
    
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