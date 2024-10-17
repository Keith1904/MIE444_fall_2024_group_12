from personnel.motorSergeant import MotorSergeant
from personnel.scout import Scout
from personnel.pathfinder import Pathfinder
from personnel.recon import Recon
from personnel.radioOperator import RadioOperator
import settings as SETTINGS
import time

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
        self.pathfinder = Pathfinder(self, self.robot)
        #self.scout = Scout(self, self.maze, self.robot)
        self.motorSergeant = MotorSergeant(self.radioOperator)
        self.recon = Recon()
        
    def execute_mission(self):
        self.recon.check_sensors(self.robot, ['u0', 'u1', 'u2', 'u3'], self.radioOperator)
        #self.scout.localize(self.robot)
        self.wall_alignment()
        #self.motorSergeant.move_along(path)
        #if self.motorSergeant.check_for_obstacles():
        #    self.motorSergeant.emergency_stop()  # Emergency stop if crash detected

    def wall_alignment(self):
        
        step_size = 2  # Small rotation step (adjust as needed)
        closest_sensor_id, previous_reading = self.recon.find_closest_sensor(self.robot)
        aligned = False
        self.motorSergeant.rotate(5)
        self.recon.check_sensors(self.robot, ['u0', 'u1', 'u2', 'u3'], self.radioOperator)
        
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
            self.recon.check_sensors(self.robot, sensor_ids, self.radioOperator)
            
            # Record the current readings
            current_readings = {sensor_id: self.robot.distance_sensors[sensor_id]["reading"] for sensor_id in sensor_ids}
            
            # Check if each sensor is at or below its previous minimum
            num_sensors_at_min = 0
            for sensor_id in sensor_ids:
                if current_readings[sensor_id] >= previous_readings[sensor_id]:
                    num_sensors_at_min += 1
                
            
            print(f"Current Readings: {current_readings}")
            print(f"Previous Readings: {previous_readings}")
            print(f"Sensors at Minimum: {num_sensors_at_min}")
            previous_readings = current_readings
            # If at least 3 sensors are at their minimum, consider it aligned
            if num_sensors_at_min >= 3:
                aligned = True

        # Fine-tune by reversing half the step size if needed
        self.motorSergeant.rotate(-step_size / 2 * direction)
        time.sleep(0.1)  # Allow time for stabilization after reversing        
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



if __name__ == "__main__":
    general = General()
    general.execute_mission() 