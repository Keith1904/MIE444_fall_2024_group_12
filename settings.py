# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[1,1,1,1,0,1,0,1],
         [1,1,0,1,1,1,1,1],
         [1,0,1,0,0,1,0,1],
         [1,1,1,1,1,1,0,1]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correct floor pattern
maze_dim_x = len(walls[0])*wall_segment_length
maze_dim_y = len(walls)*wall_segment_length


# Localization Settings
num_particles = 20

# Robot Characteristics
wheel_distance = 3              # Distance between wheels (inches)
wheel_circumference = 2.55906   # Circumference of wheels (inches)

# Sensor Settings
distance_sensors = {
    "u0": {"x": x, "y": y, "rotation": 0, "reading": 0},
    "u1": {"x": x, "y": y, "rotation": 90, "reading": 0},
    "u2": {"x": x, "y": y, "rotation": 180, "reading": 0},
    "u3": {"x": x, "y": y, "rotation": 270, "reading": 0},
    "t0": {"x": x, "y": y, "rotation": 0, "reading": 0}   
}

motor_encoders = {
    "e0": {"x": x, "y": y, "reading": 0, "previous_reading": 0},
    "e1": {"x": x, "y": y, "reading": 0, "previous_reading": 0}
}

### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds