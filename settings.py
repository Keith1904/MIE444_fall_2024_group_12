# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[3,3,1,1,0,2,0,2],
         [3,3,0,1,1,1,1,1],
         [1,0,2,0,0,1,0,1],
         [1,1,1,1,1,1,0,2]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correctfloor pattern
maze_dim_x = len(walls[0])*wall_segment_length
maze_dim_y = len(walls)*wall_segment_length

# Graphics information
frame_rate = 60             # Target frame rate (Hz)
ppi = 12                    # Number of on-screen pixels per inch on display
border_pixels = floor_segment_length * ppi  # Size of the border surrounding the maze area

background_color = (43, 122, 120)

wall_thickness = 0.25       # Thickness to draw wall segments, in inches
wall_color = (255, 0, 0)    # Tuple with wall color in (R,G,B) format

robot_thickness = 0.25      # Thickness to draw robot perimeter, in inches
robot_color = (0, 0, 255)   # Tuple with robot perimeter color in (R,G,B) format

block_thickness = 0.25      # Thickness to draw robot perimeter, in inches
block_color = (127, 127, 0) # Tuple with robot perimeter color in (R,G,B) format

# Localization Settings
num_particles = 20

# Robot Characteristics
wheel_distance = 7.3            # Distance between wheels (inches)
wheel_circumference = 2.55906   # Circumference of wheels (inches)
radius = 4.75                   # Overall radius of the robot (inches)

# Sensor Settings
distance_sensors = {
    "u0": {"x": 0, "y": 3.99, "rotation": 0, "reading": 0, "previous_reading": 0},
    "u1": {"x": 3.22, "y": 0, "rotation": 90, "reading": 0, "previous_reading": 0},
    "u2": {"x": 0, "y": -3.45, "rotation": 180, "reading": 0, "previous_reading": 0},
    "u3": {"x": 3.22, "y": 0, "rotation": -90, "reading": 0, "previous_reading": 0},
  #  "t0": {"x": 0, "y": 0, "rotation": 0, "reading": 0}   
}

motor_encoders = {
    "m0": {"x": 0, "y": 0, "reading": 0, "previous_reading": 0},
    "m1": {"x": 0, "y": 0, "reading": 0, "previous_reading": 0}
}

ir_sensor = {
    "i0": {"x": 0, "y": 0, "reading": 0}
}

### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True

if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0