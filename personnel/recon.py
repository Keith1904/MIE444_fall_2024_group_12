class Recon:
    '''This class collects data from sensors and relays it to scout and pathfinder.'''
    def __init__():


class Sensor:
    '''Generic sensor class'''
    def __init__(self, x, y, id):
        self.x = x
        self.y = y
        self.id = id
        
        
class UltrasonicSensor(Sensor):
    '''Class for ultrasonic sensor.'''
    def __init__(self, x, y, id, rotation):
        super().__init__(x, y, id)
        self.rotation = rotation
        
    def sense():
        