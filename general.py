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
        self.scout = Scout(2500, self.MAZE, self.robot, update_type = "distance", localization_type = "particle_filter", maze_walls = SETTINGS.walls)
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
            self.radioOperator.broadcast("s:0")
            time.sleep(2)
            self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
            if self.scout.localization_type == "particle_filter":
                self.scout.update_weights(self.MAZE, self.robot, sigma = 0.4)
            else:
                self.scout.predict_belief()
            while True:
                if self.motorSergeant.reset:
                    print("resetting")
                    if False:
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
                    if self.robot.distance_sensors["u0"]["reading"] > 3:
                        self.motorSergeant.drive(2)
                        time.sleep(1)
                    self.motorSergeant.rotate(direction)
                    time.sleep(1)
                    self.motorSergeant.reset = False
                    self.motorSergeant.reset_cooldown = 3
                    self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
                else:
                    self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
                    self.motorSergeant.adjust(self.robot, self.scout.localized)
                if self.scout.localization_type == "particle_filter":
                    self.scout.predict()
                    self.scout.update_weights(self.MAZE, self.robot, sigma = 1)
                    neff = self.scout.compute_neff()
                    print(f"neff: {neff}")
                    if neff < 1250:
                        print("resampling!")
                        self.scout.resample()
                    self.scout.weighted_average()
                    self.update_maze()
                else:
                    self.scout.predict_belief()
                    self.scout.update_belief()
                    estimated_position = self.scout.estimate_position()
                #self.update_objective()
                if not self.motorSergeant.reset:
                    self.motorSergeant.drive(3)
                    time.sleep(2)
                    self.motorSergeant.reset_cooldown -= 1
        if self.mode == 'manual':
            self.update_maze()
            self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
            self.scout.update_weights(self.MAZE, self.robot, sigma = 0.5)
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
                right_rotation_step = 3
                left_rotation_step = 3
                
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
        if self.scout.localization_type == "particle_filter":
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
        else:
            for x in range(32):
                for y in range(16):
                    # Get the maximum belief for this grid cell (across all possible orientations)
                    belief_value = np.max(self.scout.belief[y][x])

                    # Map belief_value to color intensity (for visualization)
                    # You can choose a color range: Low belief -> light, high belief -> dark
                    color_intensity = int(255 * belief_value)  # Darker color for higher belief

                    # Map the belief value to a color (could use a gradient here)
                    belief_color = (color_intensity, 0, 255 - color_intensity)  # Green to Red gradient

                    # Convert grid (x, y) to screen position
                    screen_x = int(round(SETTINGS.border_pixels + x * SETTINGS.ppi * 3))
                    screen_y = int(round(SETTINGS.border_pixels + y * SETTINGS.ppi * 3))

                    # Draw a rectangle representing the belief of the grid cell
                    pygame.draw.rect(self.canvas, belief_color, pygame.Rect(screen_x, screen_y, 3 * SETTINGS.ppi, 3 * SETTINGS.ppi))

            # Draw the average position of belief (green dot)
            # Compute the average belief position by finding the max belief in the grid
            max_belief_position = np.unravel_index(np.argmax(self.scout.belief), self.scout.belief.shape[:2])
            avg_x, avg_y = max_belief_position
            avg_theta = self.scout.average_theta  # This can be based on your belief or set manually

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
                   self.courier()
                   self.radioOperator.broadcast("L0:00")
                   self.courier()
                   self.objective = "dp"
            if self.objective == "dp":
                if current_location == self.dropoff_point:
                    print("Arrived at dropoff point!")
                    self.radioOperator.broadcast("s:0")
                    self.motorSergeant.drive(-2)
                    time.sleep(2)
                    self.radioOperator.broadcast("D0:00")
    


    #Block Detection and Pick-up System
    def courier(self):
        # '''Checks for the block, picks up the block, checks if the block is on board. If the block isn't detected after 3 attempts, '''
        block = False  #Intialize a boolean value for block detection. If True, block is visible from the front of the robot.
        while_count = 0  # Initialize a counter to track the number of while loop iterations

        if self.objective == "lz":  # Check if the robot is at the loading zone - might need to change syntax to reflect this (ask keith)
            #print("Arrived at the loading zone. Searching for block...")
            self.radioOperator.broadcast("s:0")
            time.sleep(2)
            while while_count <= 3:  # Attempt up to 3 times to find the block
                while_count += 1  # Increment the attempt counter
                #print(f"Attempt {while_count} to detect the block.")
                direction = 1
                if self.scout.localized:
                    current_location = (self.scout.average_x // 12, self.scout.average_y // 12)
                    if current_location == (2, 0) or current_location == (1, 0):
                        direction = -1
                block = self.block_detection(direction = -1)  # Call block detection logic
                self.recon.check_sensors(self.robot, ['u0','u1', 'u2', 'u3', 'u4', 'u5', 'm0', 'm1'], self.radioOperator)
                self.scout.predict()
                self.scout.update_weights(self.MAZE, self.robot, sigma = 0.4)
                neff = self.scout.compute_neff()
                print(f"neff: {neff}")
                if neff < 1250:
                    print("resampling!")
                    self.scout.resample()
                self.scout.weighted_average()
                self.update_maze()
                if block == True:  # If block is detected
                    #print("Block detected! Initiating pickup sequence.")
                    #self.block_pickup()     # Call block pickup logic

                    if self.block_pickup():  # Call block pickup logic
                        print("Block successfully picked up. Heading to delivery zone.")
                        return  # Exit function after success
                    else:
                        print("Failed to pick up block. Retrying...")
                else:
                    print("Block not detected. Retrying...")

            # After n unsuccessful attempts, proceed to the delivery zone
            print("Block not detected after many attempts. Heading to delivery zone.")
        else:
            print("Robot is not at the loading zone. Aborting courier operation.")



    def block_detection(self, direction = 1):
        """ Checks for the presence of the block. returns boolean block. True if block is visible by robot. """

        direction = -1
        #check readings for both front sensors, u0 is top sensor, u6 is the block facing sensor
        block = False
        while not block:
            self.recon.check_sensors(self.robot, ['u0','u6'], self.radioOperator)
            # Set the block boolean to True if the block is in front of the robot (if u0 value > u6 value )
            if self.robot.distance_sensors["u0"]["reading"] - 5 > self.robot.distance_sensors["u6"]["reading"]:
                print("Block Detected.")
                block = True  
            else:
                self.motorSergeant.rotate(3 * direction)
                time.sleep(0.3)
                

        return block 

    
    def block_pickup(self):
        """ tries 3 times to pick up the block, on the fourth try it releases the servo arm and aborts. 
        returns boolean called parcel, True if block is on board. """
        block_distance = 2.5 #Ideal distance between the robot and the block while actuating the servo (this is right when )
        servo_angle = 180  #set a servo target angle to close behind the block
        max_attempts = 3  # Maximum number of attempts to pick up the block
        attempt = 0  # Counter to track attempts
        while attempt <= max_attempts:
            self.recon.check_sensors(self.robot, ['u0','u6'], self.radioOperator)
            u6 = self.robot.distance_sensors["u6"]["reading"]  # Get the sensor reading
            
            if u6 <= block_distance:
                # Actuate the servo arm to grasp the block
                self.radioOperator.broadcast(f"s:{servo_angle}")
                time.sleep(5)
                print(f"Servo arm reached {servo_angle} degrees")
                
                # Check if the block is secured
                if u6 <= 1.6:
                    #print("Block on board!")
                    return True  # Block successfully picked up

                else:
                    # Turn to adjust position and check again
                    print("Adjusting position...")
                    self.radioOperator.broadcast("r0:-15")  # Turn robot slightly
            else:
                # Move closer to the block
                offset = u6 - block_distance
                self.radioOperator.broadcast(f"w0:{offset}")
                print(f"Moving {offset} in towards the block")
                time.sleep(3)
                self.block_detection()

            attempt += 1  # Increment attempt counter

        # If max attempts reached, open the servo arm and abort
        print("Max attempts reached. Releasing servo arm.")
        self.radioOperator.broadcast("s:00")  # open the servo arm by returning it to zero degrees
        return False  # Block pickup failed










            






    
        
        
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
    #print(general.radioOperator.broadcast("u0,u1,u2,u3,u4,u5,m0,m1", response = True))
    #general.courier()
    general.execute_mission()