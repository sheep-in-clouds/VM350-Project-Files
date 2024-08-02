import RPi.GPIO as GPIO
import time
from utils.AngleMeter.AngleMeterAlpha import AngleMeterAlpha
import cv2
from picamera2 import Picamera2
import numpy as np

class SensorControl:
    def __init__(self, ultrasonic_pins = None, camera_params = None, hsv_params = None):
        self.ultrasonic_pins = ultrasonic_pins
        self.imu = AngleMeterAlpha()
        self.camera_params = camera_params
        self.hsv_params = hsv_params
        # self.color_sensor_pins = color_sensor_pins
        self.setup_sensors()


        # Initialize camera
        if self.camera_params is not None:
            self.picam2 = Picamera2()
            self.picam2.preview_configuration.main.size = (self.camera_params['dispW'], self.camera_params['dispH'])
            self.picam2.preview_configuration.main.format = "RGB888"
            self.picam2.preview_configuration.controls.FrameRate = 30
            self.picam2.preview_configuration.align()
            self.picam2.configure("preview")
            self.picam2.start()

    def setup_sensors(self):
        GPIO.setmode(GPIO.BOARD)
        if self.ultrasonic_pins:
            for pin in self.ultrasonic_pins.values():
                GPIO.setup(pin['trig'], GPIO.OUT)
                GPIO.setup(pin['echo'], GPIO.IN)
        # self.setup_color_sensor()
        self.imu.measure()

    def setup_color_sensor(self):
        s2, s3, out = self.color_sensor_pins['s2'], self.color_sensor_pins['s3'], self.color_sensor_pins['out']
        GPIO.setup(out, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(s2, GPIO.OUT)
        GPIO.setup(s3, GPIO.OUT)

    def read_color_value(self, a0, a1):
        s2, s3, out = self.color_sensor_pins['s2'], self.color_sensor_pins['s3'], self.color_sensor_pins['out']
        GPIO.output(s2, a0)
        GPIO.output(s3, a1)

        # Give the sensor some time to adjust
        time.sleep(0.1)

        # Wait for a full cycle (this will make sure we only count full cycles)
        GPIO.wait_for_edge(out, GPIO.FALLING)
        GPIO.wait_for_edge(out, GPIO.RISING)

        start = time.time()

        GPIO.wait_for_edge(out, GPIO.FALLING)

        # The time that passed while we were waiting for the out to change
        return (time.time() - start) * 1000000

    def read_color(self):
        time.sleep(0.1)
        r = self.read_color_value(GPIO.LOW, GPIO.LOW)
        time.sleep(0.1)
        g = self.read_color_value(GPIO.HIGH, GPIO.HIGH)
        time.sleep(0.1)
        b = self.read_color_value(GPIO.LOW, GPIO.HIGH)
        return {'r': r, 'g': g, 'b': b}

    def read_ultrasonic_distance(self, trig_pin, echo_pin):
        # setup the trigger pin
        GPIO.output(trig_pin,0)
        time.sleep(2E-6)
        GPIO.output(trig_pin,1)
        time.sleep(10E-6)
        GPIO.output(trig_pin,0)
        
        timeout = time.time() + 0.1
        # measure with the echo pin
        while GPIO.input(echo_pin)==0:
            if time.time() > timeout:
                return -1
        echoStartTime = time.time()
        
        timeout = time.time() + 0.1
        while GPIO.input(echo_pin)==1:
            if time.time() > timeout:
                return -1
        echoStopTime = time.time()
        pingTravelTime = echoStopTime - echoStartTime
        echoTravelDistance = pingTravelTime * 34300
        distance = echoTravelDistance/2
        return distance # in cm

    def read_front_distance(self):
        result = self.read_ultrasonic_distance(self.ultrasonic_pins['front']['trig'], self.ultrasonic_pins['front']['echo'])
        if result == -1:
            return self.read_front_distance()
        else:
            return result

    def read_left_distance(self):
        result = self.read_ultrasonic_distance(self.ultrasonic_pins['left']['trig'], self.ultrasonic_pins['left']['echo'])
        if result == -1:
            return self.read_left_distance()
        else:
            return result

    def read_right_distance(self):
        result = self.read_ultrasonic_distance(self.ultrasonic_pins['right']['trig'], self.ultrasonic_pins['right']['echo'])
        if result == -1:
            return self.read_right_distance()
        else:
            return result
        
    def read_imu_pitch(self):
        return self.imu.get_kalman_pitch()

    def read_imu_roll(self):
        return self.imu.get_kalman_roll()

    def read_camera_data(self):
        frame = self.picam2.capture_array()
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        GhueLow, GhueHigh, GsatLow, GsatHigh, GvalLow, GvalHigh = self.hsv_params['G']
        RhueLow, RhueHigh, RsatLow, RsatHigh, RvalLow, RvalHigh = self.hsv_params['R']

        GlowerBound = np.array([GhueLow, GsatLow, GvalLow])
        GupperBound = np.array([GhueHigh, GsatHigh, GvalHigh])
        GmyMask = cv2.inRange(frameHSV, GlowerBound, GupperBound)

        RlowerBound = np.array([RhueLow, RsatLow, RvalLow])
        RupperBound = np.array([RhueHigh, RsatHigh, RvalHigh])
        RmyMask = cv2.inRange(frameHSV, RlowerBound, RupperBound)

        Gcontours, _ = cv2.findContours(GmyMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        Rcontours, _ = cv2.findContours(RmyMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = None
        if Gcontours and Rcontours:
            Gcontours = sorted(Gcontours, key=lambda x: cv2.contourArea(x), reverse=True)
            Gcontour = Gcontours[0]
            x1, y1, w1, h1 = cv2.boundingRect(Gcontour)

            Rcontours = sorted(Rcontours, key=lambda x: cv2.contourArea(x), reverse=True)
            Rcontour = Rcontours[0]
            x2, y2, w2, h2 = cv2.boundingRect(Rcontour)

            if x2 + w2 / 2 > x1 + w1 / 2:
                result = 'green_left'
            elif x2 + w2 / 2 < x1 + w1 / 2:
                result = 'green_right'

        return result






