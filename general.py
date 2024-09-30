from personnel.motorSergeant import MotorSergeant
from personnel.scout import Scout
from personnel.sentry import Sentry
from personnel.recon import Recon
from personnel.radioOperator import RadioOperator


class General:
    '''Interfaces with all other classes and coordinates all operations.'''
    
    def __init__(self):
        self.radioOperator = RadioOperator(self)
        self.motorSergeant = MotorSergeant(self)
        self.scout = Scout(self)
        self.sentry = Sentry(self)
        self.recon = Recon(self)
        self.maze = Maze()
        self.robot = Robot()
        
    def execute_mission(self):
        data = self.recon.collect_data()
        location = self.scout.find_path(data)
        self.motorSergeant.move_to(location)
        if self.sentry.check_for_obstacles():
            self.motorSergeant.emergency_stop()  # Emergency stop if crash detected
            
class Maze:
    '''Defines static characteristics of the maze.'''
    def __init__(self):
        
class Robot:
    