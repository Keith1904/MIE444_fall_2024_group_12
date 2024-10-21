import time
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
                    self.wall_alignment()
                    distance, direction = self.pathfinder.find_furthest_distance(self.robot)
                    self.motorSergeant.rotate(direction)
                    time.sleep(0.1)
                    while self.motorSergeant.movement_in_progress(self.robot):
                        time.sleep(0.3)
                    self.motorSergeant.drive(distance - 1)
                    self.motorSergeant.reset = False
                    time.sleep(0.5)
                if self.motorSergeant.movement_in_progress(self.robot):
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
        step_size = 2  # Small rotation step (adjust as needed)
        closest_sensor_id, previous_reading = self.recon.find_closest_sensor(self.robot)
        aligned = False
        self.motorSergeant.rotate(5)
        
        if self.robot.distance_sensors[closest_sensor_id]["reading"] < previous_reading:
            direction = 1
        else:
            direction = -1
        
        previous_readings = {sensor_id: float('inf') for sensor_id in ['u0', 'u1', 'u2', 'u3']}
        while not aligned:
            self.motorSergeant.rotate(step_size * direction)
            time.sleep(0.1)  # Small delay to stabilize sensor reading
            
            # Get readings for all sensors
            sensor_ids = ['u0', 'u1', 'u2', 'u3']
            time.sleep(0.1)
            # Record the current readings
            current_readings = {sensor_id: self.robot.distance_sensors[sensor_id]["reading"] for sensor_id in sensor_ids}
            
            # Check if each sensor is at or below its previous minimum
            num_sensors_at_min = 0
            for sensor_id in sensor_ids:
                if current_readings[sensor_id] >= previous_readings[sensor_id]:
                    num_sensors_at_min += 1
                
            previous_readings = current_readings
            # If at least 3 sensors are at their minimum, consider it aligned
            if num_sensors_at_min >= 3:
                aligned = True

        # Fine-tune by reversing half the step size if needed
        self.motorSergeant.rotate(-step_size / 2 * direction)
        time.sleep(0.1)  # Allow time for stabilization after reversing
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
    general.execute_mission()
    #general.wall_alignment()