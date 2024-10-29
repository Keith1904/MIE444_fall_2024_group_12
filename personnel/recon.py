import time
class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
        
    def check_sensors(self, robot, sensor_ids, radioOperator, window_size=1):
        readings, time_rx = radioOperator.broadcast(",".join(sensor_ids))
        for reading in readings:
            if reading:
                sensor_id = reading[0]
                new_value = float(reading[1])

                if sensor_id[0] in ["u", "t", "i"]:
                    # Add new reading to the list of previous readings
                    if "history" not in robot.distance_sensors[sensor_id]:
                        robot.distance_sensors[sensor_id]["history"] = []
                    robot.distance_sensors[sensor_id]["history"].append(new_value)
                    
                    # Trim history to window size
                    if len(robot.distance_sensors[sensor_id]["history"]) > window_size:
                        robot.distance_sensors[sensor_id]["history"].pop(0)
                    
                    # Calculate moving average
                    avg_value = sum(robot.distance_sensors[sensor_id]["history"]) / len(robot.distance_sensors[sensor_id]["history"])

                    # Update previous and current readings
                    robot.distance_sensors[sensor_id]["previous_reading"] = robot.distance_sensors[sensor_id]["reading"]
                    robot.distance_sensors[sensor_id]["reading"] = avg_value

                elif sensor_id[0] == "m":
                    robot.motor_encoders[sensor_id]["previous_reading"] = robot.motor_encoders[sensor_id]["reading"]
                    robot.motor_encoders[sensor_id]["reading"] = new_value
            else:
                print(f"Reading failed on {sensor_id}! Skipping update.")
        
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
        
        