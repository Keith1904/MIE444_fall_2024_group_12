import time

class Pathfinder:
    '''This class is responsible for determine the route for the robot to travel while avoiding obstacles. Also performs the block finding algorithm.'''
    def __init__(self, general, robot):
        self.general = general
        self.movement_in_progress = False
        self.movement_buffer = []
        self.robot = robot
        
    def chart_path(self):
        if not self.movement_in_progress:
            self.wall_alignment()
    
    def wall_alignment(self):
        
        step_size = 2  # Small rotation step (adjust as needed)
        closest_sensor_id, previous_reading = self.find_closest_sensor(self.robot)
        aligned = False
        self.general.motorSergeant.rotate(5)
        self.general.recon.check_sensors(self.robot, ['u0', 'u1', 'u2', 'u3'], self.general.radioOperator)
        
        if self.robot.distance_sensors[closest_sensor_id]["reading"] < previous_reading:
            direction = 1
        else:
            direction = -1
        
        previous_reading = self.robot.distance_sensors[closest_sensor_id]["reading"]
        while not aligned:
            self.general.motorSergeant.rotate(step_size * direction)
            time.sleep(0.1)  # Small delay to stabilize sensor reading
            
            # Take multiple readings and average them
            readings = []
            for _ in range(3):  # Take 3 readings (adjust as needed)
                self.general.recon.check_sensors(self.robot, ['u0', 'u1', 'u2', 'u3'], self.general.radioOperator)
                readings.append(self.robot.distance_sensors[closest_sensor_id]["reading"])
                time.sleep(0.05)  # Short delay between readings
            
            current_reading = sum(readings) / len(readings)
            print(f"Average Reading: {current_reading}, Previous Reading: {previous_reading}")

            # If the averaged reading is greater, the robot has passed the optimal alignment point
            if current_reading >= previous_reading:
                aligned = True
            else:
                previous_reading = current_reading
        self.general.motorSergeant.rotate(-step_size * direction / 2)
        
               
    def find_closest_sensor(self, robot):
        min_reading = float('inf')
        closest_sensor_id = None

        # Iterate through each sensor in the robot's distance sensors
        for sensor_id, sensor_data in robot.distance_sensors.items():
            # Get the reading from the current sensor
            reading = sensor_data["reading"]

            # Check if this reading is the smallest one found so far
            if reading < min_reading and sensor_id[0] == "u":
                min_reading = reading
                closest_sensor_id = sensor_id

        return closest_sensor_id, min_reading