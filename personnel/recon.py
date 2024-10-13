class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
        
    def check_sensors(self, robot, sensor_ids, radioOperator):
        readings = radioOperator.transmit(sensor_ids)
        for i, sensor_id in enumerate(sensor_ids):
            
            if sensor_id[0] == "u" or sensor_id[0] == "t" or sensor_id[0] == "i":
                robot.distance_sensors[sensor_id]["reading"] = readings[i]
            
            elif sensor_id[0] == "e":
                robot.motor_encoders[sensor_id]["previous_reading"] = robot.motor_encoders[sensor_id]["reading"]
                robot.motor_encoders[sensor_id]["reading"] = readings[i]
        
# class Sensor:
#     '''Generic sensor class'''
#     def __init__(self, sensor_id, x, y):
#         self.x = x
#         self.y = y
#         self.sensor_id = sensor_id
#         self.reading = 0

        
# class DistanceSensor(Sensor):
#     '''Class for ultrasonic and TOF sensors.'''
#     def __init__(self, sensor_id, x, y, rotation):
#         super().__init__(sensor_id, x, y)
#         self.rotation = rotation
        
# class MotorEncoder(Sensor):
#     '''Class for motor encoders'''
#     def __init__(self, sensor_id, x, y):
#         super().__init__(sensor_id, x, y)
#         self.previous_reading = 0
    
#     def sense(self, radioOperator):
#         self.previous_reading = self.reading
#         self.reading = radioOperator.transmit(self.sensor_id)
        