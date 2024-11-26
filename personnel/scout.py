import random
import numpy as np
import math
import pygame
import scipy.stats
from collections import Counter
from statistics import mean
import utilities
from sklearn.cluster import DBSCAN

class Scout:
    '''This class performs the particle filter localization algorithm.'''
    
    def __init__(self, num_particles, maze, robot, update_type = "distance"):
        # Define MCL Parameters
        self.num_particles = num_particles
        self.maze = maze
        self.particles = self.initialize_particles()
        self.robot = robot
        self.average_x = 0
        self.average_y = 0
        self.average_theta = 0
        self.localized = False
        self.update_type = update_type
    
    def initialize_particles(self):
        """Randomly generate the first batch of particles around the maze in valid locations."""
        particles = []
        cardinal_angles = [0, 90, 180, 270]
        
        for i in range(self.num_particles):
            while True:
                x = random.uniform(0, self.maze.size_x)
                y = random.uniform(0, self.maze.size_y)
                theta = random.choice(cardinal_angles)  # Choose a cardinal direction
                
                if self.is_valid_position(x, y) and self.is_clear_of_walls(x, y):
                    particles.append(Particle(x, y, theta, 1 / self.num_particles))
                    break
        return particles

    def is_clear_of_walls(self, x, y):
        """Checks if the particle's position is at least three inches from any wall."""
        clearance = 3  # Minimum clearance in inches
        x_cell = int(x / 12)
        y_cell = int(y / 12)
        
        # Check cells within three inches in each direction for walls
        for dx in [-clearance, 0, clearance]:
            for dy in [-clearance, 0, clearance]:
                neighbor_x = x + dx
                neighbor_y = y + dy
                
                # Convert to maze cell indices
                neighbor_x_cell = int(neighbor_x / 12)
                neighbor_y_cell = int(neighbor_y / 12)
                
                # Verify neighbor is within bounds and not a wall
                if 0 <= neighbor_x_cell < self.maze.size_x / 12 and 0 <= neighbor_y_cell < self.maze.size_y / 12:
                    if self.maze.orig_walls[neighbor_y_cell][neighbor_x_cell] == 0:
                        return False
        return True
    
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
        distance_noise_std = 1  # Adjust based on your observations
        angle_noise_std = 5     # Adjust based on your observations

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
            
def update_weights(self, maze, robot, sigma=0.3, epsilon=1e-8):
    """
    Update the weight of each particle based on whether there is a wall detected or not.

    :param maze: Maze object containing the layout and wall positions.
    :param robot: Robot object providing actual sensor readings.
    :param sigma: Standard deviation of sensor noise for fallback Gaussian update.
    :param epsilon: A small value to prevent division by zero during normalization.
    """
    total_weight = 0.0
    sensor_ids = ["u0", "u1", "u2", "u3", "u4", "u5"]
    wall_threshold = 4.0  # Distance threshold for detecting a wall

    for particle in self.particles:
        # Initialize weight to a default value
        particle.weight = 1.0

        # Check if the particle is in a valid position
        if not (self.is_valid_position(particle.x, particle.y) and self.is_clear_of_walls(particle.x, particle.y)):
            particle.weight = epsilon
        else:
            if self.update_type == "distance":
                # Compare wall presence between simulated and actual readings
                for sensor_id in sensor_ids:
                    simulated_reading = particle.simulate_ultrasonic_sensor(maze, robot, sensor_id)
                    actual_reading = robot.distance_sensors[sensor_id]["reading"]

                    # Check for wall presence
                    simulated_wall = simulated_reading < wall_threshold
                    actual_wall = actual_reading < wall_threshold

                    if simulated_wall == actual_wall:
                        # If both readings agree, give higher weight
                        particle.weight *= 1.0
                    else:
                        # Penalize if they disagree
                        particle.weight *= epsilon
            elif self.update_type == "wall":
                # Compare wall presence between simulated and actual readings
                for sensor_id in sensor_ids:
                    simulated_reading = particle.simulate_ultrasonic_sensor(maze, robot, sensor_id)
                    actual_reading = robot.distance_sensors[sensor_id]["reading"]

                    # Check for wall presence
                    simulated_wall = simulated_reading < wall_threshold
                    actual_wall = actual_reading < wall_threshold

                    if simulated_wall == actual_wall:
                        # If both readings agree, give higher weight
                        particle.weight *= 1.0
                    else:
                        # Penalize if they disagree
                        particle.weight *= 0.5

        # Accumulate total weight for normalization
        total_weight += particle.weight

    # Normalize weights
    if total_weight < epsilon:
        # Avoid collapse by resetting weights if all become too small
        for particle in self.particles:
            particle.weight = 1.0 / len(self.particles)
    else:
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
                 
    def resample(self, injection_fraction=0.05):
        """
        Resample particles with replacement based on their weights, with a fraction of random particles injected.

        :param injection_fraction: Fraction of new particles to inject (default 10%).
        :param maze_bounds: Dictionary with maze boundaries for random particle injection.
                            Example: {'x_min': 0, 'x_max': 10, 'y_min': 0, 'y_max': 10}.
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

        # Inject random particles
        num_to_inject = int(N * injection_fraction)
        for _ in range(num_to_inject):
            while True:
                x = random.uniform(0, self.maze.size_x)
                y = random.uniform(0, self.maze.size_y)
                theta = random.choice([0, 90, 180, 270])  # Choose a cardinal direction
                if self.is_valid_position(x, y):
                    random_particle = Particle(x, y, theta, 1 / self.num_particles)
                    new_particles[random.randint(0, N - 1)] = random_particle  # Replace an existing particle
                    break

        self.particles = new_particles  # Replace old particles with the new ones
        if self.is_localized():
            self.localized = True
            print("Localized!")
        else:
            self.localized = False

    def copy_particle(self, particle):
        """
        Create a deep copy of a particle.
        Adjust this function based on the actual attributes of your particle.
        """
        return Particle(particle.x, particle.y, particle.theta, particle.weight)

    def compute_neff(self):
        """
        Calculate the effective sample size (Neff) of the particles based on their weights.

        :return: The effective sample size (Neff).
        """
        sum_of_squares = sum(particle.weight ** 2 for particle in self.particles)
        if sum_of_squares == 0:  # Avoid division by zero
            return 0
        return 1.0 / sum_of_squares

    def weighted_average(self):
        total_weight = sum(particle.weight for particle in self.particles)

        # Avoid division by zero
        if total_weight == 0:
            print("Warning: Total weight is zero. Adjusting weights to avoid collapse.")
            total_weight = len(self.particles)  # fallback to average without weights

        # Compute weighted sums for x and y
        weighted_x = sum(particle.x * particle.weight for particle in self.particles) / total_weight
        weighted_y = sum(particle.y * particle.weight for particle in self.particles) / total_weight

        # Track previous position to calculate vector
        if hasattr(self, 'average_x') and hasattr(self, 'average_y'):
            previous_x, previous_y = self.average_x, self.average_y
        else:
            previous_x, previous_y = weighted_x, weighted_y  # Initialize to current if no previous value exists

        # Calculate the vector and its orientation angle
        delta_x = weighted_x - previous_x
        delta_y = weighted_y - previous_y
        weighted_theta = math.atan2(delta_y, delta_x)

        # Update the stored average position
        self.average_x = weighted_x
        self.average_y = weighted_y
        self.average_theta = weighted_theta

    def position_standard_deviation(self):


        # Compute averages for x and y
        x = sum(particle.x for particle in self.particles) / len(self.particles)
        y = sum(particle.y for particle in self.particles) / len(self.particles)

        # Calculate weighted variance of the Euclidean distances
        distance_variance = sum(
            ((particle.x - x) ** 2 + (particle.y - y) ** 2)
            for particle in self.particles
        ) / (len(self.particles) - 1)

        # Standard deviation as the square root of the variance
        std_dev_distance = math.sqrt(distance_variance)

        return std_dev_distance

    def is_localized(self, eps=6, min_samples=500):
        positions = np.array([[p.x, p.y] for p in self.particles])
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(positions)
        labels = clustering.labels_
        largest_cluster = max(set(labels), key=list(labels).count)
        cluster_size = list(labels).count(largest_cluster)
        print(f"CLUSTER SIZE: {cluster_size}")
        return cluster_size / len(self.particles) > 0.85  # Example threshold
    
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
        
        walls_to_check = maze.reduced_walls
        squared_distance = 100^2
        for wall in walls_to_check:
            collision_points = utilities.collision(beam, wall)
            if not collision_points:
                pass
            else:
                beam[1], squared_distance = utilities.closest_fast([sensor_x, sensor_y], collision_points)
        return math.sqrt(squared_distance)