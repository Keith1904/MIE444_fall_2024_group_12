import numpy as np
import random

class Scout:
    '''This class performs the localization algorithm and instructs the motorSergeant where to go.'''
    
    def __init__(self, num_particles, maze):
        # Define MCL Parameters
        self.num_particles = num_particles
        self.initialize_particles(maze_data)
        self.weight = np.ones(num_particles) / num_particles
        self.maze = maze
    
    def initialize_particles(self, map_data):
        """Randomly generate the first batch of particles around the maze in valid locations."""
        self.particles = []
        for i in range(self.num_particles):
            while True:
                x = random.uniform(0, self.maze.maze_dim_x)
                y = random.uniform(0, self.maze.maze_dim_y)
                
                if self.is_valid_position(x, y):
                    self.particles.append(Particle(x, y))
                    break
    
    def is_valid_position(self, x, y):
        """Checks if the location of a particle is valid."""
        
        # Convert x and y positions to indices in the walls attribute.
        x_cell = int(x / 12)
        y_cell = int(y / 12)
        
        # Check that the position is within the maze boundaries. 
        if 0 <= x_cell < self.maze.maze_dim_x / 12 and 0 <= y_cell < self.maze.maze_dim_y / 12:
            return self.maze.walls[y_cell][x_cell] == 1
        else:
            return False
        
                
                

class Particle:
    '''Defines the attributes of a particle including it's position and weight.'''
    
    def __init__(self, x, y, weight=1.0):
        self.x = x          # Particle's x position
        self.y = y          # Particle's y position
        self.weight = weight # Particle's weight (probability)
    
    def update_weight(self, new_weight):
        """Update the weight of the particle."""
        self.weight = new_weight