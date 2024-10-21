
class Pathfinder:
    '''This class is responsible for determine the route for the robot to travel while avoiding obstacles. Also performs the block finding algorithm.'''
   
    def find_furthest_distance(self, robot):
        # Get readings for all sensors
        sensor_ids = ['u0', 'u1', 'u2', 'u3']
        
        # Record the current readings
        current_readings = {sensor_id: robot.distance_sensors[sensor_id]["reading"] for sensor_id in sensor_ids}
        
        # Check if all sensors except 'u2' are less than 12
        other_sensors_below_12 = all(current_readings[sensor_id] < 12 for sensor_id in sensor_ids if sensor_id != 'u2')
        
        if other_sensors_below_12:
            # If the condition is met, return u1's distance and direction
            furthest_distance = current_readings['u2']
            furthest_direction = robot.distance_sensors['u2']['rotation']
        else:
            # Find the sensor with the maximum reading, excluding 'u2'
            non_u2_readings = {sensor_id: reading for sensor_id, reading in current_readings.items() if sensor_id != 'u2'}
            furthest_sensor_id = max(non_u2_readings, key=non_u2_readings.get)
            furthest_distance = non_u2_readings[furthest_sensor_id]
            furthest_direction = robot.distance_sensors[furthest_sensor_id]["rotation"]
        
        return furthest_distance, furthest_direction