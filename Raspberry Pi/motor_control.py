import RPi.GPIO as GPIO
import time

class MotorControl:
    def __init__(self, left_motor_pins, right_motor_pins, transform_motor_pins):
        self.left_motor_pins = left_motor_pins
        self.right_motor_pins = right_motor_pins
        self.transform_motor_pins = transform_motor_pins
        self.setup_motors()

    def setup_motors(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.left_motor_pins['in1'], GPIO.OUT)
        GPIO.setup(self.left_motor_pins['in2'], GPIO.OUT)

        GPIO.setup(self.right_motor_pins['in1'], GPIO.OUT)
        GPIO.setup(self.right_motor_pins['in2'], GPIO.OUT)

        GPIO.setup(self.transform_motor_pins['in1'], GPIO.OUT)
        GPIO.setup(self.transform_motor_pins['in2'], GPIO.OUT)

    def move_forward(self):
        GPIO.output(self.left_motor_pins['in1'], GPIO.HIGH)
        GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in1'], GPIO.HIGH)
        GPIO.output(self.right_motor_pins['in2'], GPIO.LOW)

    def move_backward(self):
        GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.left_motor_pins['in2'], GPIO.HIGH)
        GPIO.output(self.right_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in2'], GPIO.HIGH)

    def turn_left(self):
        GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in1'], GPIO.HIGH)
        GPIO.output(self.right_motor_pins['in2'], GPIO.LOW)

    def turn_right(self):
        GPIO.output(self.left_motor_pins['in1'], GPIO.HIGH)
        GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in2'], GPIO.LOW)

    def stop(self):
        GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.right_motor_pins['in2'], GPIO.LOW)

    def extend_wheels(self):
        GPIO.output(self.transform_motor_pins['in1'], GPIO.HIGH)
        GPIO.output(self.transform_motor_pins['in2'], GPIO.LOW)

    def retract_wheels(self):
        GPIO.output(self.transform_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.transform_motor_pins['in2'], GPIO.HIGH)

    def stop_transform_motor(self):
        GPIO.output(self.transform_motor_pins['in1'], GPIO.LOW)
        GPIO.output(self.transform_motor_pins['in2'], GPIO.LOW)

    def cleanup(self):
        GPIO.cleanup()

    def extend_wheels_safe(self):
        self.extend_wheels()
        time.sleep(0.9)
        self.stop_transform_motor()

    def retract_wheels_safe(self):
        self.retract_wheels()
        time.sleep(0.9)
        self.stop_transform_motor()
        
    def move_forward_time(self, t):
        time.sleep(0.02)
        self.move_forward()
        time.sleep(t)
        self.stop()
        time.sleep(0.02)
        
    def move_backward_time(self, t):
        time.sleep(0.02)
        self.move_backward()
        time.sleep(t)
        self.stop()
        time.sleep(0.02)
        
    def turn_left_time(self, t):
        time.sleep(0.02)
        self.turn_left()
        time.sleep(t)
        self.stop()
        time.sleep(0.02)
        
    def turn_right_time(self, t):
        time.sleep(0.02)
        self.turn_right()
        time.sleep(t)
        self.stop()
        time.sleep(0.02)
# class MotorControl:
#     def __init__(self, left_motor_pins, right_motor_pins, transform_motor_pins):
#         self.left_motor_pins = left_motor_pins
#         self.right_motor_pins = right_motor_pins
#         self.transform_motor_pins = transform_motor_pins
#         self.setup_motors()

#     def setup_motors(self):
#         GPIO.setmode(GPIO.BOARD)
#         GPIO.setup(self.left_motor_pins['pwm'], GPIO.OUT)
#         GPIO.setup(self.left_motor_pins['in1'], GPIO.OUT)
#         GPIO.setup(self.left_motor_pins['in2'], GPIO.OUT)

#         GPIO.setup(self.right_motor_pins['pwm'], GPIO.OUT)
#         GPIO.setup(self.right_motor_pins['in1'], GPIO.OUT)
#         GPIO.setup(self.right_motor_pins['in2'], GPIO.OUT)

#         GPIO.setup(self.transform_motor_pins['pwm'], GPIO.OUT)
#         GPIO.setup(self.transform_motor_pins['in1'], GPIO.OUT)
#         GPIO.setup(self.transform_motor_pins['in2'], GPIO.OUT)

#         self.left_pwm = GPIO.PWM(self.left_motor_pins['pwm'], 1000)
#         self.right_pwm = GPIO.PWM(self.right_motor_pins['pwm'], 1000)
#         self.transform_pwm = GPIO.PWM(self.transform_motor_pins['pwm'], 1000)

#         self.left_pwm.start(0)
#         self.right_pwm.start(0)
#         self.transform_pwm.start(0)

#     def move_forward(self, speed):
#         self.left_pwm.ChangeDutyCycle(speed)
#         self.right_pwm.ChangeDutyCycle(speed)
#         GPIO.output(self.left_motor_pins['in1'], GPIO.HIGH)
#         GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
#         GPIO.output(self.right_motor_pins['in1'], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins['in2'], GPIO.LOW)

#     def move_backward(self, speed):
#         self.left_pwm.ChangeDutyCycle(speed)
#         self.right_pwm.ChangeDutyCycle(speed)
#         GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.left_motor_pins['in2'], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.right_motor_pins['in2'], GPIO.HIGH)

#     def turn_left(self, speed):
#         self.left_pwm.ChangeDutyCycle(speed)
#         self.right_pwm.ChangeDutyCycle(speed)
#         GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.left_motor_pins['in2'], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins['in1'], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins['in2'], GPIO.LOW)

#     def turn_right(self, speed):
#         self.left_pwm.ChangeDutyCycle(speed)
#         self.right_pwm.ChangeDutyCycle(speed)
#         GPIO.output(self.left_motor_pins['in1'], GPIO.HIGH)
#         GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
#         GPIO.output(self.right_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.right_motor_pins['in2'], GPIO.HIGH)

#     def stop(self):
#         self.left_pwm.ChangeDutyCycle(0)
#         self.right_pwm.ChangeDutyCycle(0)
#         GPIO.output(self.left_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.left_motor_pins['in2'], GPIO.LOW)
#         GPIO.output(self.right_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.right_motor_pins['in2'], GPIO.LOW)

#     def extend_wheels(self, speed):
#         self.transform_pwm.ChangeDutyCycle(speed)
#         GPIO.output(self.transform_motor_pins['in1'], GPIO.HIGH)
#         GPIO.output(self.transform_motor_pins['in2'], GPIO.LOW)

#     def retract_wheels(self, speed):
#         self.transform_pwm.ChangeDutyCycle(speed)
#         GPIO.output(self.transform_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.transform_motor_pins['in2'], GPIO.HIGH)

#     def stop_transform_motor(self):
#         self.transform_pwm.ChangeDutyCycle(0)
#         GPIO.output(self.transform_motor_pins['in1'], GPIO.LOW)
#         GPIO.output(self.transform_motor_pins['in2'], GPIO.LOW)

#     def cleanup(self):
#         self.left_pwm.stop()
#         self.right_pwm.stop()
#         self.transform_pwm.stop()
#         GPIO.cleanup()

# if __name__ == "__main__":
#     left_motor_pins = {'pwm': 18, 'in1': 23, 'in2': 24}
#     right_motor_pins = {'pwm': 19, 'in1': 27, 'in2': 22}
#     transform_motor_pins = {'pwm': 20, 'in1': 5, 'in2': 6}

#     motor_control = MotorControl(left_motor_pins, right_motor_pins, transform_motor_pins)

#     try:
#         motor_control.move_forward(50)
#         time.sleep(2)
#         motor_control.stop()

#         motor_control.turn_left(50)
#         time.sleep(2)
#         motor_control.stop()

#         motor_control.extend_wheels(50)
#         time.sleep(2)
#         motor_control.stop_transform_motor()

#         motor_control.retract_wheels(50)
#         time.sleep(2)
#         motor_control.stop_transform_motor()

#     except KeyboardInterrupt:
#         motor_control.cleanup()
