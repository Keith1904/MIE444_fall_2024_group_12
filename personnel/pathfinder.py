import math

class Pathfinder:
    '''This class is responsible for determine the route for the robot to travel while avoiding obstacles. Also performs the block finding algorithm.'''
    def __init__(self, walls):
        self.walls = walls
    
    def find_furthest_distance(self, robot):
        # Get readings for all sensors
        sensor_ids = ['u0', 'u1', 'u2', 'u3', 'u4', 'u5']
        
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
    

    def get_turn_angle(self, current_location, current_location_radians, goal_location):
        # Define directions and corresponding vectors
        directions = ['east', 'south', 'west', 'north']
        direction_vectors = {'north': (0, -1), 'east': (1, 0), 'south': (0, 1), 'west': (-1, 0)}
        
        # Normalize current_radian_direction to be between 0 and 2π
        current_radian_direction = current_location_radians % (2 * math.pi)
        
        # Convert inches to cell coordinates
        current_location = (int(current_location[0] // 12), int(current_location[1] // 12))
        print(f"current location {current_location}")
        # Convert the normalized radian direction to one of the four compass directions
        direction_index = int((current_radian_direction + math.pi / 4) // (math.pi / 2)) % 4
        current_direction = directions[direction_index]
        print(f"current direction: {current_direction}")
        # Calculate row and column differences between current and goal locations
        x_diff = goal_location[0] - current_location[0]
        y_diff = goal_location[1] - current_location[1]
        
        # Determine potential directions towards the goal
        if abs(x_diff) > abs(y_diff):
            primary_direction = 'east' if x_diff > 0 else 'west'
        else:
            primary_direction = 'south' if y_diff > 0 else 'north'
        
        # Define directions to check, ordered by closeness to the goal
        possible_directions = [primary_direction]
        if primary_direction in ['north', 'south']:
            possible_directions.append('east' if x_diff > 0 else 'west')
        else:
            possible_directions.append('south' if y_diff > 0 else 'north')
        print(f"possible directions: {possible_directions}")
        # Check for obstacles and select a valid direction
        for direction in possible_directions:
            dx, dy = direction_vectors[direction]
            new_col = current_location[0] + dx
            new_row = current_location[1] + dy
            # Ensure the new location is within bounds and not an obstacle
            if 0 <= new_row < len(self.walls) and 0 <= new_col < len(self.walls[0]) and self.walls[new_row][new_col] != 0:
                desired_direction = direction
                break
        else:
            # If no valid primary/secondary direction found, keep the current direction
            if current_direction == 'north':
                desired_direction = 'south'
            elif current_direction == 'east':
                desired_direction = 'west'
            elif current_direction == 'south':
                desired_direction = 'north'
            elif current_direction == 'west':
                desired_direction = 'east'
        
        # Calculate angle to turn based on the current and desired directions
        current_index = directions.index(current_direction)
        desired_index = directions.index(desired_direction)
        angle_diff = (desired_index - current_index) * 90
        
        # Ensure angle is between -270 and 270
        angle = (angle_diff + 360) % 360  # Adjust to positive angle
        if angle > 180:
            angle -= 360  # Adjust to negative if greater than 180
        
        return angle