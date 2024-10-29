import numpy as np
import random
import math

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
        
        for particle in self.particles:
            #print(particle.x)
            #print(particle.y)
            #print(particle.theta)
            # Update the bearing in degrees
            theta_new = particle.theta + math.degrees(delta_theta)
            # Normalize theta to stay within 0 to 360 degrees
            theta_new = theta_new % 360
            
            if delta_theta == 0:
                # Moving in a straight line
                x_new = particle.x + d * math.cos(math.radians(particle.theta))
                y_new = particle.y - d * math.sin(math.radians(particle.theta))  # Subtract to account for downward y-axis
            else:
                # Moving in an arc
                R = d / delta_theta  # Convert delta_theta to radians for calculation
                x_new = particle.x - R * math.sin(math.radians(particle.theta)) + R * math.sin(math.radians(theta_new))
                y_new = particle.y + R * math.cos(math.radians(particle.theta)) - R * math.cos(math.radians(theta_new))
                y_new = particle.y - (R * math.cos(math.radians(particle.theta)) - R * math.cos(math.radians(theta_new)))  # Account for downward y-axis
            
            # Update the particle's position
            particle.update_position(x=x_new, y=y_new, theta=theta_new)
                 

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
        
    def simulate_ultrsonic_sensor(self):
        