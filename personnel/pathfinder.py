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
        closest_sensor_id, min_reading = self.find_closest_sensor(self.robot)
        self.general.motorSergeant.move("r0:360")
        while( True):
            self.general.recon.check_sensors(self.robot, ['u0', 'u1', 'u2', 'u3', 'm0', 'm1', 'i0'], self.general.radioOperator)
            #self.scout.localize(self.robot)
            
        
               
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