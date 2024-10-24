import time
import math
import threading
import keyboard
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
        #self.scout = Scout(self, self.maze, self.robot)
        self.motorSergeant = MotorSergeant(self.radioOperator)
        self.recon = Recon()
        self.mode = "auto"
        self.sensor_thread = threading.Thread(target=self.recon.check_sensors, args = (self.robot, ['u0', 'u1', 'u2', 'u3', 'm0', 'm1'], self.radioOperator), daemon=True)
        self.manual_control_thread = threading.Thread(target=self.manual_control, daemon=True)
        self.sensor_thread.start()
        self.manual_control_thread.start()
        self.last_input = ''
        
        
    def execute_mission(self):
        #self.scout.localize(self.robot)
        while True:
            if self.mode == 'auto':
                if self.motorSergeant.reset:
                    print("resetting")
                    #self.wall_alignment()
                    distance, direction = self.pathfinder.find_furthest_distance(self.robot)
                    self.motorSergeant.rotate(direction)
                    time.sleep(0.5)
                    while self.motorSergeant.movement_in_progress(self.robot):
                        time.sleep(0.3)
                    print("sending it")
                    self.motorSergeant.drive(distance - 1)
                    self.motorSergeant.reset = False
                    self.motorSergeant.reset_cooldown = self.robot.motor_encoders["m0"]["reading"]
                    time.sleep(0.5)
                elif self.motorSergeant.movement_in_progress(self.robot):
                    self.motorSergeant.check_for_collision(self.robot)
                    time.sleep(0.1)
                else:
                    time.sleep(0.5)
                    if not self.motorSergeant.movement_in_progress(self.robot):
                        self.motorSergeant.reset = True
            else:
                time.sleep(0.5)

    def wall_alignment(self):
        print("Aligning with wall...")
        time.sleep(1)
        step_distance = 1  # Move forward by 1 inch (adjust as needed)
        # Get initial readings from left and right sensors (u1 and u3)
        initial_left = self.robot.distance_sensors['u1']["reading"]
        initial_right = self.robot.distance_sensors['u3']["reading"]
        if self.robot.distance_sensors['u0']["reading"] > self.robot.distance_sensors['u2']["reading"]:
            # Move forward a short distance
            direction = 1
            self.motorSergeant.drive(direction * step_distance)
            time.sleep(4)  # Small delay for the robot to stabilize after moving
        else:
            direction = -1
            self.motorSergeant.drive(direction * step_distance)
            time.sleep(4)  # Small delay for the robot to stabilize after moving
            
        
        # Get new readings from left and right sensors (u1 and u3)
        new_left = self.robot.distance_sensors['u1']["reading"]
        new_right = self.robot.distance_sensors['u3']["reading"]
        
        # Calculate the differences in distances
        left_diff = new_left - initial_left
        right_diff = new_right - initial_right
        print(new_left)
        print(initial_left)
        # Calculate the deviation angle using arctan
        if abs(left_diff) < abs(right_diff) and abs(left_diff) < 1:
            print(left_diff)
            deviation_angle = math.degrees(math.atan(direction * (left_diff) / step_distance))
        elif right_diff < 1:
            deviation_angle = math.degrees(-math.atan(direction * (right_diff) / step_distance))

        # Rotate the robot to correct the alignment
        if abs(deviation_angle) > 5:  # Threshold to detect misalignment (adjust as needed)
            print(f"Deviation detected: {deviation_angle:.2f} degrees")

            if deviation_angle > 0:
                print("Rotating clockwise to align...")
                self.motorSergeant.rotate(deviation_angle)
            else:
                print("Rotating counterclockwise to align...")
                self.motorSergeant.rotate(deviation_angle)
            

        print("Alignment done!") 
    
    def manual_control(self):
        while True:
            if self.mode == 'manual':
                if keyboard.is_pressed('w'):
                    if self.last_input != 'w':
                        self.motorSergeant.stop()
                        self.motorSergeant.drive(100)
                        self.last_input = 'w' 
                elif keyboard.is_pressed('a'):
                    if self.last_input != 'a':
                        self.motorSergeant.stop()
                        self.motorSergeant.rotate(-360)
                        self.last_input = 'a'
                elif keyboard.is_pressed('s'):
                    if self.last_input != 's':
                        self.motorSergeant.stop()
                        self.motorSergeant.drive(-100)
                        self.last_input = 's'
                elif keyboard.is_pressed('d'):
                    if self.last_input != 'd':
                        self.motorSergeant.stop()
                        self.motorSergeant.rotate(360)
                        self.last_input = 'd'
                else:
                    if self.last_input != '':
                        self.motorSergeant.stop()
                        self.last_input = ''
            # Check for mode switching keys
            if keyboard.is_pressed('m'):
                self.mode = 'manual'
                print("Switched to manual mode.")
                time.sleep(0.1)  # Small delay to avoid multiple detections
            elif keyboard.is_pressed('p'):
                self.mode = 'auto'
                print("Switched to autonomous mode.")
                time.sleep(0.1)

            time.sleep(0.1)  # Prevent excessive CPU usage

            
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
        self.x = None
        self.y = None
        self.direction = None
        self.radius = SETTINGS.radius



if __name__ == "__main__":
    general = General()
    general.wall_alignment()
    general.execute_mission()