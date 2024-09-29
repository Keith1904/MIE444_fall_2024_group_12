from operator import Operator
from scout import Scout
from sentry import Sentry
from recon import Recon


class General:
    '''Interfaces with all other classes and coordinates all operations.'''
    
    def __init__(self):
        self.operator = Operator()
        self.scout = Scout()
        self.sentry = Sentry()
        self.recon = Recon()
        
    def execute_mission(self):
        data = self.recon.collect_data()
        location = self.scout.find_path(data)
        self.operator.move_to(location)
        if self.sentry.check_for_obstacles():
            self.operator.emergency_stop()  # Emergency stop if crash detected
        data = self.recon.gather_data()