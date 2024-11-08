import math
import numpy as np
import time

class MotorSergeant:
    '''This class issues all motion commands.'''
    def __init__(self, radioOperator):
        self.radioOperator = radioOperator
        self.reset = True
        self.reset_cooldown = 0
        
    def drive(self, distance):
        print(f"Driving this distance: {distance}")
        self.stop()
        self.radioOperator.broadcast("w0:" + str(distance))

    def rotate(self, angle):
        print(f"Rotating this angle: {angle}")
        self.stop()
        self.radioOperator.broadcast("r0:" + str(angle))

    def issue_command(self, cmd):
        self.stop()
        self.radioOperator.broadcast(cmd)

    def stop(self):
        self.radioOperator.broadcast("xx")
        
    def movement_in_progress(self, robot):
        prev_m0 = robot.motor_encoders["m0"]["previous_reading"]
        prev_m1 = robot.motor_encoders["m1"]["previous_reading"]
        curr_m0 = robot.motor_encoders["m0"]["reading"]
        curr_m1 = robot.motor_encoders["m1"]["reading"]
        
        if curr_m0 != prev_m0 or curr_m1 != prev_m1:
            return True
        return False
    
    def adjust(self, robot):
        distance_sensors_copy = robot.distance_sensors.copy()
        motor_encoders_copy = robot.motor_encoders.copy()
        for sensor_id, sensor_data in distance_sensors_copy.items():
            reading = sensor_data["reading"]
            sensor_distance = math.sqrt(sensor_data['x']**2 + sensor_data['y']**2)
            if reading < 3 and sensor_id == 'u0':
                self.reset = True
            if reading  < 2:
                if sensor_id == 'u4':
                    #self.drive(-0.5)
                    self.rotate(-5)
                    break
                elif sensor_id == 'u5':
                    #self.drive(-0.5)
                    self.rotate(5)
                    break
                if sensor_id == "u0":
                    self.drive(-0.5)
                    self.reset = True
                    break
            elif 2 < reading < 6:
                if sensor_id == "u4" and reading + 0.2 > distance_sensors_copy['u1']['reading']:
                    #self.drive(-0.5)
                    self.rotate(5)
                    break
                elif sensor_id == "u5" and reading + 0.2 > distance_sensors_copy['u3']['reading']:
                    #self.drive(-0.5)              
                    self.rotate(-5)
                    break
            elif reading > distance_sensors_copy["u0"]["reading"] + 8 and self.reset_cooldown <= 0:
                if distance_sensors_copy["u1"]["reading"] > distance_sensors_copy["u0"]["reading"] + 8 and distance_sensors_copy["u4"]["reading"] > distance_sensors_copy["u0"]["reading"] + 8 or distance_sensors_copy["u3"]["reading"] > distance_sensors_copy["u0"]["reading"] + 8 and distance_sensors_copy["u5"]["reading"] > distance_sensors_copy["u0"]["reading"] + 8:
                    self.reset = True
                    break