import time
class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
        
    def check_sensors(self, robot, sensor_ids, radioOperator):
        while True:
            radioOperator.broadcast(",".join(sensor_ids))
            readings, time_rx = radioOperator.receive()
            for reading in readings:
                if reading:
                    sensor_id = reading[0]      
                    if sensor_id[0] == "u" or sensor_id[0] == "t" or sensor_id[0] == "i":
                        robot.distance_sensors[sensor_id]["previous_reading"] = robot.distance_sensors[sensor_id]["reading"]
                        robot.distance_sensors[sensor_id]["reading"] = float(reading[1])
                    elif sensor_id[0] == "m":
                        robot.motor_encoders[sensor_id]["previous_reading"] = robot.motor_encoders[sensor_id]["reading"]
                        robot.motor_encoders[sensor_id]["reading"] = float(reading[1])
                else:
                    print(f"Reading failed on {sensor_id}! Skipping update." )
            time.sleep(0.1)
    
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
        
        