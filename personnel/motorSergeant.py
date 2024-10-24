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
        print(f"received this: {distance}")
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
        
    def movement_in_progress(self, robot):
        prev_m0 = robot.motor_encoders["m0"]["previous_reading"]
        prev_m1 = robot.motor_encoders["m1"]["previous_reading"]
        curr_m0 = robot.motor_encoders["m0"]["reading"]
        curr_m1 = robot.motor_encoders["m1"]["reading"]
        
        if curr_m0 != prev_m0 or curr_m1 != prev_m1:
            return True
        return False
    
    def check_for_collision(self, robot):
        distance_sensors_copy = robot.distance_sensors.copy()
        motor_encoders_copy = robot.motor_encoders.copy()
        for sensor_id, sensor_data in distance_sensors_copy.items():
            reading = sensor_data["reading"]
            sensor_distance = math.sqrt(sensor_data['x']**2 + sensor_data['y']**2)
            if reading + sensor_distance - robot.radius < 1 and reading <= sensor_data["previous_reading"]:
                print("Obstacle detected! Stopping...")
                self.stop()
                time.sleep(0.1)
                if sensor_id == 'u1':
                    delta_sensor = sensor_data["reading"] - sensor_data["previous_reading"]
                    delta_encoder = motor_encoders_copy["m0"]["reading"] - motor_encoders_copy["m0"]["previous_reading"]
                    print(f"delta_sensor: {abs(delta_sensor)}")
                    print(f"delta_encoder: {delta_encoder}")
                    angle_deviation = np.arctan(delta_sensor / delta_encoder)
                    self.rotate(-5)
                    time.sleep(0.2)
                    while self.movement_in_progress(robot):
                        time.sleep(0.1)
                    self.drive(robot.distance_sensors["u0"]["reading"] - 1)
                    time.sleep(0.2)
                    break
                elif sensor_id == 'u3':
                    delta_sensor = sensor_data["reading"] - sensor_data["previous_reading"]
                    delta_encoder = motor_encoders_copy["m0"]["reading"] - motor_encoders_copy["m0"]["previous_reading"]
                    print(f"delta_sensor: {delta_sensor}")
                    print(f"delta_encoder: {delta_encoder}")
                    angle_deviation = np.arctan(delta_sensor / delta_encoder)
                    self.rotate(5)
                    time.sleep(0.2)
                    while self.movement_in_progress(robot):
                        time.sleep(0.1)
                    self.drive(robot.distance_sensors["u0"]["reading"] - 1)
                    time.sleep(0.5)
                    break
                self.reset = True
                break
            elif 2 < reading + sensor_distance - robot.radius < 5 and reading >= sensor_data["previous_reading"]:
                if sensor_id == "u1":
                    self.rotate(5)
                    time.sleep(0.2)
                    while self.movement_in_progress(robot):
                        time.sleep(0.1)
                    self.drive(robot.distance_sensors["u0"]["reading"] - 1)
                    time.sleep(0.2)
                    break
                elif sensor_id == "u3":
                    self.rotate(-5)
                    time.sleep(0.2)
                    while self.movement_in_progress(robot):
                        time.sleep(0.1)
                    self.drive(robot.distance_sensors["u0"]["reading"] - 1)
                    time.sleep(0.2)
                    break
            elif reading > distance_sensors_copy["u0"]["reading"] + 8 and (sensor_id == "u1" or sensor_id == "u3") and motor_encoders_copy["m0"]["reading"] > self.reset_cooldown + 8:
                print(self.reset_cooldown)
                print(motor_encoders_copy["m0"]["reading"])
                self.stop()
                time.sleep(0.1)
                self.drive(2)
                print("im here")
                time.sleep(0.3)
                while self.movement_in_progress(robot):
                    time.sleep(0.1)
                self.reset = True
      #          self.reset_cooldown = robot.motor_encoders["m0"]["reading"]
                break