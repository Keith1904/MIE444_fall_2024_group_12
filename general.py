import time
import math
import threading
import numpy as np
import sys
import shapely as shp
import pygame
import keyboard
from personnel.motorSergeant import MotorSergeant
from personnel.scout import Scout
from personnel.pathfinder import Pathfinder
from personnel.recon import Recon
from personnel.radioOperator import RadioOperator
from maze import Maze
import settings as SETTINGS

class General:
    '''Interfaces with all other classes and coordinates all operations.'''
    
    def __init__(self):
        self.MAZE = Maze()
        self.robot = Robot(SETTINGS.distance_sensors, SETTINGS.motor_encoders, SETTINGS.ir_sensor)
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
        self.pathfinder = Pathfinder()
        self.scout = Scout(2000, self.MAZE, self.robot)
        self.motorSergeant = MotorSergeant(self.radioOperator)
        self.recon = Recon()
        time.sleep(3)
        self.mode = "manual"
        self.last_input = ''
        self.dropoff_point = (0, 0)
        
        
    def execute_mission(self):
        if self.mode == 'auto':
            self.update_maze()
            self.wall_alignment()
            self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
            self.scout.update_weights(self.MAZE, self.robot, sigma = 0.2)
            while True:
                self.update_maze()
                if self.motorSergeant.reset:
                    print("resetting")
                    distance, direction = self.pathfinder.find_furthest_distance(self.robot)
                    self.motorSergeant.rotate(direction)
                    time.sleep(5)
                    self.motorSergeant.reset = False
                    self.motorSergeant.reset_cooldown = 3
                else:
                    self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5'], self.radioOperator)
                    self.motorSergeant.adjust(self.robot)
                    time.sleep(1)
                self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
                self.scout.predict()
                self.scout.update_weights(self.MAZE, self.robot, sigma = 0.2)
                neff = self.scout.compute_neff()
                if neff < 500:
                    print("resampling!")
                    self.scout.resample()
                if not self.motorSergeant.reset:
                    self.motorSergeant.drive(2)
                    self.motorSergeant.reset_cooldown -= 1
                    time.sleep(2)
        if self.mode == 'manual':
            self.update_maze()
            self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
            self.scout.update_weights(self.MAZE, self.robot, sigma = 0.2)
            while True:
                self.update_maze()
                print("Enter a command: ")
                command = input()
                if command == "align":
                    self.wall_alignment()
                else:
                    self.radioOperator.broadcast(command)
                    time.sleep(5)
                self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
                self.scout.predict()
                self.scout.update_weights(self.MAZE, self.robot, sigma = 0.2)
                neff = self.scout.compute_neff()
                if neff < 500:
                    print("resampling!")
                    self.scout.resample()
                    
                
            
                
                

    def wall_alignment(self):
        print("Aligning with wall...")
        
        # Define a tolerance for alignment
        tolerance = 0.1  # Adjust as needed for your setup
        
        while True:
            # Get readings from the sensors
            self.recon.check_sensors(self.robot, ['u1', 'u3', 'u4', 'u5'], self.radioOperator)
            
            # Calculate the differences for each side
            right_diff = self.robot.distance_sensors['u1']['reading'] - self.robot.distance_sensors['u4']['reading']
            left_diff = self.robot.distance_sensors['u3']['reading'] - self.robot.distance_sensors['u5']['reading']
            
            # Check if the robot is aligned on both sides
            if abs(right_diff) < tolerance or abs(left_diff) < tolerance:
                print("Wall alignment successful.")
                print(f"Right diff: {right_diff}, Left diff: {left_diff}")
                break
            else:
                print(f"Aligning... Right diff: {right_diff}, Left diff: {left_diff}")
                
                # Determine rotation direction
                if right_diff > tolerance:
                    # Rotate left to align the right side
                    self.motorSergeant.rotate(-5)
                elif right_diff < -tolerance:
                    # Rotate right to align the right side
                    self.motorSergeant.rotate(5)
                elif left_diff > tolerance:
                    # Rotate right to align the left side
                    self.motorSergeant.rotate(5)
                elif left_diff < -tolerance:
                    # Rotate left to align the left side
                    self.motorSergeant.rotate(-5)
                
                time.sleep(0.3)   

    def initialize_maze(self):
        self.MAZE.import_walls()
        self.MAZE.generate_floor()
        CANVAS_WIDTH = self.MAZE.size_x * SETTINGS.ppi + SETTINGS.border_pixels * 2
        CANVAS_HEIGHT = self.MAZE.size_y * SETTINGS.ppi + SETTINGS.border_pixels * 2
        pygame.init()
        self.canvas = pygame.display.set_mode([CANVAS_WIDTH, CANVAS_HEIGHT])
      
    def update_maze(self):
        game_events = pygame.event.get()
        keypress = pygame.key.get_pressed()
        self.canvas.fill(SETTINGS.background_color)

        # Draw the maze checkerboard pattern
        self.MAZE.draw_floor(self.canvas)

        # Draw the maze walls
        self.MAZE.draw_walls(self.canvas)
        
        # Draw the particles
        particle_color = (0, 0, 255)  # Color for the particles (red in this case)
        particle_radius = 5  # Radius of each particle's circle
        
        for particle in self.scout.particles:  # Assuming self.particles is a list of (x, y) tuples
            particle_int_pos = (int(round(SETTINGS.border_pixels + particle.x * SETTINGS.ppi)), 
                                int(round(SETTINGS.border_pixels + particle.y * SETTINGS.ppi)))
            
            # Calculate the radius based on the particle's weight
            # You might want to define a scaling factor for the weight to radius mapping
            #min_radius = 5  # Minimum particle radius
            #max_radius = 10  # Maximum particle radius
            #weight_scale = 5  # Adjust this value to scale the effect of weight on radius
            
            pygame.draw.circle(self.canvas, particle_color, particle_int_pos, particle_radius)

            # Flip the display (update the canvas)
            pygame.display.flip()
    
        
        
class Robot:
    def __init__(self, distance_sensors, motor_encoders, ir_sensor):
        self.distance_sensors = distance_sensors
        self.motor_encoders = motor_encoders
        self.ir_sensor = ir_sensor
        self.x = None
        self.y = None
        self.direction = None
        self.radius = SETTINGS.radius
        self.wheel_distance = SETTINGS.wheel_distance



if __name__ == "__main__":
    general = General()
    np.random.seed(SETTINGS.floor_seed)
    general.initialize_maze()
    general.execute_mission()