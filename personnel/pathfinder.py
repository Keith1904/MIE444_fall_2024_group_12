import time

class Pathfinder:
    '''This class is responsible for determine the route for the robot to travel while avoiding obstacles. Also performs the block finding algorithm.'''
    def __init__(self, general, robot):
        self.general = general
        self.movement_in_progress = False
        self.movement_buffer = []
        self.robot = robot