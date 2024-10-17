

class MotorSergeant:
    '''This class issues all motion commands.'''
    def __init__(self, radioOperator):
        self.radioOperator = radioOperator
        
    def drive(self, distance):
        self.stop()
        self.radioOperator.broadcast("w0:" + str(distance))

    def rotate(self, angle):
        self.stop()
        self.radioOperator.broadcast("r0:" + str(angle))

    def stop(self):
        self.radioOperator.broadcast("xx")