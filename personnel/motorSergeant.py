

class MotorSergeant:
    '''This class issues all motion commands.'''
    def __init__(self, radioOperator):
        self.radioOperator = radioOperator
        self.movement_in_progress = False
        
    def drive(self, distance):
        self.stop()
        self.radioOperator.broadcast("w0:" + str(distance))

    def rotate(self, angle):
        self.stop()
        self.radioOperator.broadcast("r0:" + str(angle))
    
    def issue_command(self, cmd):
        self.stop()
        self.radioOperator.broadcast(cmd)

    def stop(self):
        self.radioOperator.broadcast("xx")