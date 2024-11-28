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
        response = ""
        while not response:
            response = self.radioOperator.broadcast("w0:" + str(distance), response = True)

    def rotate(self, angle):
        print(f"Rotating this angle: {angle}")
        remaining_angle = angle
        while abs(remaining_angle) > 90:
            # Break into chunks of ±90
            chunk_angle = 90 if remaining_angle > 0 else -90
            print(f"Rotating chunk angle: {chunk_angle}")
            response = ""
            while not response:
                response = self.radioOperator.broadcast("r0:" + str(chunk_angle), response=True)
                time.sleep(1)
            remaining_angle -= chunk_angle

        # Rotate the remaining angle (if any)
        if remaining_angle != 0:
            print(f"Rotating final angle: {remaining_angle}")
            response = ""
            while not response:
                response = self.radioOperator.broadcast("r0:" + str(remaining_angle), response=True)
                time.sleep(0.5)

    def issue_command(self, cmd):
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
    
    def adjust(self, robot, localized):
        distance_sensors_copy = robot.distance_sensors.copy()
        motor_encoders_copy = robot.motor_encoders.copy()
        for sensor_id, sensor_data in distance_sensors_copy.items():
            reading = sensor_data["reading"]
            sensor_distance = math.sqrt(sensor_data['x']**2 + sensor_data['y']**2)
            if reading < 3 and sensor_id == 'u0':
                self.reset = True
            if reading  < 2:
                if sensor_id == 'u4':
                    self.drive(-0.5)
                    time.sleep(0.5)
                    self.rotate(-10)
                    print("u4 too small")
                    break
                elif sensor_id == 'u5':
                    self.drive(-0.5)
                    time.sleep(0.5)
                    self.rotate(10)
                    print("u5 too small")
                    break
                if sensor_id == "u0":
                    self.drive(-1)
                    self.reset = True
                    print("u0 too small")
                    break
            elif 2 < reading < 6:
                if sensor_id == "u4" and reading - 0.2 > distance_sensors_copy['u1']['reading']:
                    #self.drive(-0.5)
                    self.rotate(10)
                    print("aligning right side")
                    break
                elif sensor_id == "u5" and reading - 0.2 > distance_sensors_copy['u3']['reading']:
                    #self.drive(-0.5)              
                    self.rotate(-10)
                    print("aligning left side")
                    break
            elif reading > 10 and sensor_id == "u1" or sensor_id == "u3":
                if ((distance_sensors_copy["u1"]["reading"] > 10 and distance_sensors_copy["u4"]["reading"] > 10) or (distance_sensors_copy["u3"]["reading"] > 10 and distance_sensors_copy["u5"]["reading"] > 10)) and localized and self.reset_cooldown <= 0:
                    self.reset = True
                    print("reset right side")
                    break
                
            elif reading > distance_sensors_copy["u0"]["reading"] + 8 and self.reset_cooldown <= 0:
                if (distance_sensors_copy["u1"]["reading"] > distance_sensors_copy["u0"]["reading"] + 6 and distance_sensors_copy["u4"]["reading"] > distance_sensors_copy["u0"]["reading"]) or (distance_sensors_copy["u3"]["reading"] > distance_sensors_copy["u0"]["reading"] + 6 and distance_sensors_copy["u5"]["reading"] > distance_sensors_copy["u0"]["reading"]) or (distance_sensors_copy["u1"]["reading"] > distance_sensors_copy["u0"]["reading"] and distance_sensors_copy["u4"]["reading"] > distance_sensors_copy["u0"]["reading"] + 6) or (distance_sensors_copy["u3"]["reading"] > distance_sensors_copy["u0"]["reading"] and distance_sensors_copy["u5"]["reading"] > distance_sensors_copy["u0"]["reading"] + 6):
                    self.reset = True
                    print("reset left side")
                    break