from personnel.motorSergeant import MotorSergeant
from personnel.scout import Scout
from personnel.pathfinder import Pathfinder
from personnel.recon import Recon
from personnel.radioOperator import RadioOperator
import settings as SETTINGS


class General:
    '''Interfaces with all other classes and coordinates all operations.'''
    
    def __init__(self):
        self.maze = Maze()
        self.robot = Robot()
        self.radioOperator = RadioOperator(
            HOST=SETTINGS.HOST,
            PORT_TX=SETTINGS.PORT_TX,
            PORT_RX=SETTINGS.PORT_RX,
            BAUDRATE=SETTINGS.BAUDRATE,
            PORT_SERIAL=SETTINGS.PORT_SERIAL,
            TIMEOUT_SERIAL=SETTINGS.TIMEOUT_SERIAL,
            FRAMESTART=SETTINGS.FRAMESTART,
            FRAMEEND=SETTINGS.FRAMEEND,
            CMD_DELIMITER=SETTINGS.CMD_DELIMITER,
            SIMULATE=SETTINGS.SIMULATE,
            TRANSMIT_PAUSE=SETTINGS.TRANSMIT_PAUSE
            )
        self.pathfinder = MotorSergeant()
        self.scout = Scout(self, self.maze, self.robot)
        self.motorSergeant = MotorSergeant(self, self.motorSergeant)
        self.recon = Recon()
        
    def execute_mission(self):
        self.recon.check_sensors(self.robot, ['u0', 'u1', 'u2', 'u3', 'm0', 'm1', 'i0'], self.radioOperator)
        location = self.scout.localize(self.robot)
        path = self.pathfinder.chart_path(location)
        self.motorSergeant.move_along(path)
        if self.motorSergeant.check_for_obstacles():
            self.motorSergeant.emergency_stop()  # Emergency stop if crash detected
            
class Maze:
    '''Defines static characteristics of the maze.'''
    def __init__(self):
        self.wall_segment_length = SETTINGS.wall_segment_length
        self.floor_segment_length = SETTINGS.floor_segment_length
        self.walls = SETTINGS.walls
        self.floor_seed = SETTINGS.floor_seed
        self.maze_dim_x = SETTINGS.maze_dim_x
        self.maze_dim_y = SETTINGS.maze_dim_y
        
class Robot:
    def __init__(self, distance_sensors, motor_encoders, ir_sensor):
        self.distance_sensors = distance_sensors
        self.motor_encoders = motor_encoders
        self.ir_sensor = ir_sensor



if __name__ == "__main__":
    general = General()
    general.execute_mission() 