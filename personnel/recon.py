import time
class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
        
    def check_sensors(self, robot, sensor_ids, radioOperator, window_size=1):
        readings, time_rx = radioOperator.broadcast(",".join(sensor_ids), response = True)
        while True:
            if len(readings) != len(sensor_ids):
                readings, time_rx = radioOperator.broadcast(",".join(sensor_ids), response = True)
                time.sleep(0.5)
            else:
                break
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
                print("Reading failed on a sensor! Skipping update.")
        
        time.sleep(0.3)
        
        