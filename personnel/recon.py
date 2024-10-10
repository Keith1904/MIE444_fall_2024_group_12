class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
    def __init__(self, distance_sensors, motor_encoders):
        self.distance_sensors = distance_sensors
        self.motor_encoders = motor_encoders
        
    def check_sensors(self, sensor_ids, radioOperator):
        readings = radioOperator.transmit(sensor_ids)
        for i, sensor_id in enumerate(sensor_ids):
            if sensor_id[0] == "u" or sensor_id[0] == "t":
                self.distance_sensors[sensor_id]["reading"] = readings[i]
            
            elif sensor_id[0] == "e":
                self.motor_encoders[sensor_id]["previous_reading"] = self.motor_encoders[sensor_id]["reading"]
                self.motor_encoders[sensor_id]["reading"] = readings[i]
        
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
        