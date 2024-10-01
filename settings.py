# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[1,1,1,1,0,1,0,1],
         [1,1,0,1,1,1,1,1],
         [1,0,1,0,0,1,0,1],
         [1,1,1,1,1,1,0,1]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correctfloor pattern
maze_dim_x = len(walls[0])*wall_segment_length
maze_dim_y = len(walls)*wall_segment_length


# Localization Settings
num_particles = 20