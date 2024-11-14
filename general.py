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
        self.pathfinder = Pathfinder(SETTINGS.walls)
        self.scout = Scout(2500, self.MAZE, self.robot)
        self.motorSergeant = MotorSergeant(self.radioOperator)
        self.recon = Recon()
        time.sleep(3)
        self.mode = "auto"
        self.last_input = ''
        self.objective = "lz"
        self.dropoff_point = (5, 0)
        
        
    def execute_mission(self):
        if self.mode == 'auto':
            self.update_maze()
            #self.wall_alignment()
            self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
            self.scout.update_weights(self.MAZE, self.robot, sigma = 0.2)
            while True:
                if self.motorSergeant.reset:
                    print("resetting")
                    if self.scout.localized:
                        if self.objective == "lz":
                            print("Heading to lz!")
                            direction = self.pathfinder.get_turn_angle((self.scout.average_x, self.scout.average_y), self.scout.average_theta, (0, 0))
                        elif self.objective == "dp":
                            print("Heading to dp!")
                            direction = self.pathfinder.get_turn_angle((self.scout.average_x, self.scout.average_y), self.scout.average_theta, self.dropoff_point)
                        if direction == None:
                            distance, direction = self.pathfinder.find_furthest_distance(self.robot)
                    else: 
                        distance, direction = self.pathfinder.find_furthest_distance(self.robot)
                    self.motorSergeant.drive(1)
                    time.sleep(1)
                    self.motorSergeant.rotate(direction)
                    self.motorSergeant.reset = False
                    self.motorSergeant.reset_cooldown = 3
                else:
                    self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
                    self.motorSergeant.adjust(self.robot, self.scout.localized)
                self.scout.predict()
                self.scout.update_weights(self.MAZE, self.robot, sigma = 0.4)
                neff = self.scout.compute_neff()
                print(f"neff: {neff}")
                if neff < 1250:
                    print("resampling!")
                    self.scout.resample()
                self.scout.weighted_average()
                self.update_maze()
                self.update_objective()
                if not self.motorSergeant.reset:
                    self.motorSergeant.drive(3)
                    time.sleep(1)
                    self.motorSergeant.reset_cooldown -= 1
        if self.mode == 'manual':
            self.update_maze()
            self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
            self.scout.update_weights(self.MAZE, self.robot, sigma = 0.4)
            while True:
                self.update_maze()
                print("Enter a command: ")
                command = input()
                if command == "align":
                    self.wall_alignment()
                else:
                    self.radioOperator.broadcast(command)
                    time.sleep(3)
                self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
                self.scout.predict()
                self.scout.update_weights(self.MAZE, self.robot, sigma = 0.2)
                neff = self.scout.compute_neff()
                if neff < 2500:
                    print("resampling!")
                    self.scout.resample()
                    
    def wall_alignment(self):
        print("Aligning with wall...")
        
        # Define a base tolerance for alignment
        alignment_tolerance = 0.4  # General alignment tolerance
        max_rotation_step = 3      # Maximum step size for large misalignments
        min_rotation_step = 0.5    # Minimum step size to ensure gradual adjustment
        
        while True:
            # Get readings from the sensors
            self.recon.check_sensors(self.robot, ['u1', 'u3', 'u4', 'u5'], self.radioOperator)
            
            # Calculate the differences for each side
            right_diff = self.robot.distance_sensors['u1']['reading'] - self.robot.distance_sensors['u4']['reading']
            left_diff = self.robot.distance_sensors['u3']['reading'] - self.robot.distance_sensors['u5']['reading']
            
            # Check if the robot is aligned on both sides
            if abs(right_diff) < alignment_tolerance and abs(left_diff) < alignment_tolerance:
                print("Wall alignment successful.")
                print(f"Right diff: {right_diff}, Left diff: {left_diff}")
                break
            else:
                print(f"Aligning... Right diff: {right_diff}, Left diff: {left_diff}")
                
                # Calculate scaled rotation steps based on the difference, capped between min and max
                right_rotation_step = max(min(abs(right_diff), max_rotation_step), min_rotation_step)
                left_rotation_step = max(min(abs(left_diff), max_rotation_step), min_rotation_step)
                
                # Adjust rotation direction based on the differences
                if right_diff > alignment_tolerance:
                    # Rotate left to align the right side
                    self.motorSergeant.rotate(-right_rotation_step)
                elif right_diff < -alignment_tolerance:
                    # Rotate right to align the right side
                    self.motorSergeant.rotate(right_rotation_step)
                elif left_diff > alignment_tolerance:
                    # Rotate right to align the left side
                    self.motorSergeant.rotate(left_rotation_step)
                elif left_diff < -alignment_tolerance:
                    # Rotate left to align the left side
                    self.motorSergeant.rotate(-left_rotation_step)

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
        particle_color = (0, 0, 255)  # Color for the particles (blue)
        particle_radius = 5  # Radius of each particle's circle

        for particle in self.scout.particles:
            particle_int_pos = (
                int(round(SETTINGS.border_pixels + particle.x * SETTINGS.ppi)),
                int(round(SETTINGS.border_pixels + particle.y * SETTINGS.ppi))
            )
            pygame.draw.circle(self.canvas, particle_color, particle_int_pos, particle_radius)

        # Draw the average position (green dot) and direction (small line)
        avg_x = self.scout.average_x
        avg_y = self.scout.average_y
        avg_theta = self.scout.average_theta

        # Convert average (x, y) to screen position
        avg_pos = (
            int(round(SETTINGS.border_pixels + avg_x * SETTINGS.ppi)),
            int(round(SETTINGS.border_pixels + avg_y * SETTINGS.ppi))
        )

        # Draw the green dot at the average position
        avg_color = (0, 255, 0)  # Green color
        avg_radius = 6  # Radius of the average position dot
        pygame.draw.circle(self.canvas, avg_color, avg_pos, avg_radius)

        # Calculate the end of the direction line
        line_length = 15  # Length of the direction indicator line
        
        end_pos = (
            int(avg_pos[0] + line_length * math.cos(avg_theta)),
            int(avg_pos[1] + line_length * math.sin(avg_theta))
        )

        # Draw the direction line from the center of the green dot
        pygame.draw.line(self.canvas, avg_color, avg_pos, end_pos, 2)

        # Flip the display (update the canvas)
        pygame.display.flip()
    
    def update_objective(self):
        if self.scout.localized:
            current_location = (self.scout.average_x // 12, self.scout.average_y // 12)
            if self.objective == "lz":
               if current_location == (0, 0) or current_location == (0, 1) or current_location == (1, 0) or current_location == (1, 1):
                   print("Arrived at loading zone!")
                   self.radioOperator.broadcast("L0:00")
                   self.objective = "dp"
            if self.objective == "dp":
                if current_location == self.dropoff_point:
                    print("Arrived at dropoff point!")
                    self.radioOperator.broadcast("D0:00")
    
    def block_detection(self):
        self.motorSergeant.drive()
        self.motorSergeant.rotate()
        self.recon.check_sensors(self.robot, ['u1', 'u3', 'u4', 'u5'], self.radioOperator) # u6 for TOF block

        #check if the block is in front of the robot
        self.recon.check_sensors(self.robot, ['u1','u6'])
        ##if self.robot.distance_sensors["u0"]{"reading"} > self.robot.distance_sensors["u5"]{"reading"}:




    
        
        
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