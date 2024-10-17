class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
        
    def check_sensors(self, robot, sensor_ids, radioOperator):
        radioOperator.broadcast(",".join(sensor_ids))
        readings, time_rx = radioOperator.receive()
        for reading in readings:
            sensor_id = reading[0]      
            if sensor_id[0] == "u" or sensor_id[0] == "t" or sensor_id[0] == "i":
                robot.distance_sensors[sensor_id]["reading"] = float(reading[1])
            
            elif sensor_id[0] == "e":
                robot.motor_encoders[sensor_id]["previous_reading"] = robot.motor_encoders[sensor_id]["reading"]
                robot.motor_encoders[sensor_id]["reading"] = float(reading[1])
        
        