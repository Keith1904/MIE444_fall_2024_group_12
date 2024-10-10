class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
    def __init__(self, general):
        self.general = general
        self.radioOperator = self.general.radioOperator
    
    
        


class Sensor:
    '''Generic sensor class'''
    def __init__(self, sensor_id, x, y):
        self.x = x
        self.y = y
        self.sensor_id = sensor_id
        self.reading = 0

    def sense(self, radioOperator):
        self.reading = radioOperator.transmit(self.sensor_id)
        
class DistanceSensor(Sensor):
    '''Class for ultrasonic and TOF sensors.'''
    def __init__(self, sensor_id, x, y, rotation):
        super().__init__(sensor_id, x, y)
        self.rotation = rotation
        
class MotorEncoder(Sensor):
    '''Class for motor encoders'''
    def __init__(self, sensor_id, x, y):
        super().__init__(sensor_id, x, y)
        self.previous_reading = 0
    
    def sense(self, radioOperator):
        self.previous_reading = self.reading
        self.reading = radioOperator.transmit(self.sensor_id)
        