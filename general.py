from personnel.motorSergeant import MotorSergeant
from personnel.scout import Scout
from personnel.sentry import Sentry
from personnel.recon import Recon
from personnel.radioOperator import RadioOperator
import settings as SETTINGS


class General:
    '''Interfaces with all other classes and coordinates all operations.'''
    
    def __init__(self):
        self.maze = Maze()
        self.robot = Robot()
        self.radioOperator = RadioOperator(self)
        self.motorSergeant = MotorSergeant(self, self.robot)
        self.scout = Scout(self, self.maze, self.robot)
        self.sentry = Sentry(self, self.motorSergeant)
        self.recon = Recon(self, self.robot, self.radioOperator)
        
    def execute_mission(self):
        data = self.recon.collect_data()
        location = self.scout.find_path(data)
        self.motorSergeant.move_to(location)
        if self.sentry.check_for_obstacles():
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
    