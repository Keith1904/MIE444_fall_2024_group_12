
class Pathfinder:
    '''This class is responsible for determine the route for the robot to travel while avoiding obstacles. Also performs the block finding algorithm.'''
   
    def find_furthest_distance(self, robot):
        # Get readings for all sensors
        sensor_ids = ['u0', 'u1', 'u2', 'u3']
        # Record the current readings
        current_readings = {sensor_id: robot.distance_sensors[sensor_id]["reading"] for sensor_id in sensor_ids}
        # Find the sensor with the maximum reading
        furthest_sensor_id = max(current_readings, key=current_readings.get)
        furthest_distance = current_readings[furthest_sensor_id]
        furthest_direction = robot.distance_sensors[furthest_sensor_id]["rotation"]
        return furthest_distance, furthest_direction