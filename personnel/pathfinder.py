import math
from collections import deque

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
            # If the condition is met, return u2's distance and direction
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
        
        # Normalize current_radian_direction to between 0 and 2π
        current_radian_direction = current_location_radians % (2 * math.pi)
        
        # Convert inches to cell coordinates
        current_location = (int(current_location[0] // 12), int(current_location[1] // 12))
        goal_location = (int(goal_location[0]), int(goal_location[1]))  # Goal in cell coordinates

        # Check if start and goal are in bounds and accessible
        if not (0 <= current_location[1] < len(self.walls) and 
                0 <= current_location[0] < len(self.walls[0]) and 
                0 <= goal_location[1] < len(self.walls) and 
                0 <= goal_location[0] < len(self.walls[0]) and 
                self.walls[current_location[1]][current_location[0]] != 0 and 
                self.walls[goal_location[1]][goal_location[0]] != 0):
            print("Start or goal out of bounds or in a wall.")
            return None

        # Convert radians to one of the four compass directions
        direction_index = int((current_radian_direction + math.pi / 4) // (math.pi / 2)) % 4
        current_direction = directions[direction_index]

        # BFS to find the shortest path from current_location to goal_location
        def bfs_shortest_path(start, goal):
            queue = deque([(start, [])])
            visited = set()
            
            while queue:
                (x, y), path = queue.popleft()
                if (x, y) == goal:
                    return path
                
                for direction, (dx, dy) in direction_vectors.items():
                    nx, ny = x + dx, y + dy
                    # Ensure new location is within bounds, accessible, and not visited
                    if (0 <= ny < len(self.walls) and 0 <= nx < len(self.walls[0]) and 
                        self.walls[ny][nx] != 0 and (nx, ny) not in visited):
                        queue.append(((nx, ny), path + [direction]))
                        visited.add((nx, ny))
            return None

        # Get the shortest path as a list of directions
        path = bfs_shortest_path(current_location, goal_location)
        if not path:
            print("No path found.")
            return None
        
        # Desired direction is the first step in the path
        desired_direction = path[0]
        
        # Calculate angle to turn based on the current and desired directions
        current_index = directions.index(current_direction)
        desired_index = directions.index(desired_direction)
        angle_diff = (desired_index - current_index) * 90
        
        # Ensure angle is between -270 and 270
        angle = (angle_diff + 360) % 360  # Adjust to positive angle
        if angle > 180:
            angle -= 360  # Adjust to negative if greater than 180
        
        return angle