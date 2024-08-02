from motor_control import MotorControl
from sensor_control import SensorControl
import time
import math


class TaskManager:
    def __init__(self, motor_control, sensor_control):
        self.motor_control = motor_control
        self.sensor_control = sensor_control
        self.width = 65
        self.line_num = 0
        self.start_distance = 0

    def detect_start_lines(self):

        pass

    def detect_ending_lines(self):

        pass

    def detect_distance_to_steps(self):
        return self.sensor_control.read_front_distance()

    def is_close_to_steps(self, x):
        return self.detect_distance_to_steps() < x

    def has_crossed_slope(self):
        return self.sensor_control.read_imu_pitch() > 3

    def parking(self, delta, t, isLeft):
        self.motor_control.move_forward_time(t)
        if isLeft:
            self.motor_control.turn_left_time(4.8)
        else:
            self.motor_control.turn_right_time(4.8)
        self.motor_control.move_forward_time(4)
        

    def keep_centered_in_lane(self):
        left_distance = self.sensor_control.read_left_distance()
        right_distance = self.sensor_control.read_right_distance()
        ratio = self.width / (left_distance + right_distance)
        ratio = min(1, ratio)
        angle = math.acos(ratio)
        delta = ((left_distance - right_distance) / 2) * math.cos(angle)
        angle = math.degrees(angle)
        print(f"Left Distance: {left_distance:.2f} cm Right Distance: {right_distance:.2f} Angle: {angle:.2f} du")


        if angle > 10 and (left_distance - right_distance) > 0:
            self.motor_control.turn_left()
            print("turn_left")
        elif angle > 10 and (left_distance - right_distance) < 0:
            self.motor_control.turn_right()
            print("turn_right")
        else:
            self.motor_control.move_forward()
            print("move_forward")
            
    def keep_centered_in_lane_simple(self):
        left_distance = self.sensor_control.read_left_distance()
        right_distance = self.sensor_control.read_right_distance()
        print(f"Left Distance: {left_distance:.2f} cm Right Distance: {right_distance:.2f} cm")
        if (left_distance > right_distance + 10):
            self.motor_control.turn_left()
            print("turn_left")
        elif (left_distance + 10 < right_distance):
            self.motor_control.turn_right()
            print("turn_right")
        else:
            self.motor_control.move_forward()
            print("move_forward")

    def positioning(self, t = 0.05, m = 10):
        delta = None
        for _ in range(m):
            left_distance = self.sensor_control.read_left_distance()
            right_distance = self.sensor_control.read_right_distance()
            ratio = self.width / (left_distance + right_distance)
            ratio = min(1, ratio)
            angle = math.acos(ratio)
            delta = ((left_distance - right_distance) / 2) * math.cos(angle)
            angle = math.degrees(angle)

            if angle > 3:
                if (left_distance - right_distance) > 0:
                    self.motor_control.turn_left()
                elif (left_distance - right_distance) < 0:
                    self.motor_control.turn_right()

            time.sleep(t)
        
        return delta



    def perform_task(self):
        print("Task started")
        self.motor_control.stop()
        self.motor_control.stop_transform_motor()
        self.start_distance = self.detect_distance_to_steps()
        print("1")
        print(self.start_distance)
        self.width = self.sensor_control.read_left_distance() + self.sensor_control.read_right_distance()
        print(self.width)
        print("2")
        while not self.is_close_to_steps(x=20):
            self.motor_control.move_forward()
            print(self.detect_distance_to_steps())
            time.sleep(0.05)

        self.motor_control.stop()

        self.motor_control.extend_wheels_safe()

        #while not self.has_crossed_slope():
            #self.keep_centered_in_lane()
            #time.sleep(0.05)

        #self.motor_control.stop()

        #self.motor_control.retract_wheels_safe()

        #while (self.sensor_control.read_left_distance() + self.sensor_control.read_right_distance()) < self.width + 10:
        #    self.keep_centered_in_lane()
        #    time.sleep(0.1)

        while not self.has_crossed_slope():
            self.motor_control.move_forward()
            print(self.sensor_control.read_imu_pitch())
            time.sleep(0.05)
        print("3")
        self.motor_control.stop()
        self.motor_control.retract_wheels_safe()
        turn_direction = self.sensor_control.read_camera_data()
        self.motor_control.move_forward_time(14)
        
        isLeft = None
        if turn_direction == "green_left":
            self.motor_control.turn_left_time(4.8)
            isLeft = True
        else:
            self.motor_control.turn_right_time(4.8)
            isLeft = False
        
        #self.motor_control.move_forward_time(3)
        #while (self.sensor_control.read_left_distance() + self.sensor_control.read_right_distance()) < self.width + 10:
        #    self.keep_centered_in_lane()
        #    time.sleep(0.1)
        
        #time.sleep(0.5)
        #self.motor_control.stop()
        
        # delta_ = self.positioning()

        # while not self.detect_ending_lines:
        #     self.motor_control.move_forward()
        #     time.sleep(0.1)

        # self.parking(delta=self.line_num)
        #self.start_distance = 10
        if self.start_distance > 95:
            self.parking(0, 11, isLeft)
        elif self.start_distance > 70:
            self.parking(0, 13, isLeft)
        else:
            self.parking(0, 15, isLeft)
            
        print("Task completed")

