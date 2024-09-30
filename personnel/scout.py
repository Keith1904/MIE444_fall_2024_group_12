import numpy as np

walls = [[3,3,1,1,0,2,0,2],
         [3,3,0,1,1,1,1,1],
         [1,0,2,0,0,1,0,1],
         [1,1,1,1,1,1,0,2]]

class Scout:
    '''This class performs the localization algorithm and instructs the Operator where to go.'''
    
    def __init__(self, num_particles, maze_data):
        # Define MCL Parameters
        self.num_particles = num_particles
        self.initialize_particles(maze_data)
        self.weight = np.ones(num_particles) / num_particles
        self.maze_data = maze_data
    
    def initialize_particles(self, map_data):
        """Randomly generate the first batch of particles around the maze."""
        particles = []
        for i in range(self.num_particles):
            