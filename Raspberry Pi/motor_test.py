from motor_control import MotorControl
from sensor_control import SensorControl
from task_manager import TaskManager
import RPi.GPIO as GPIO
import time


def run_test():
    motor_control.stop()
    motor_control.stop_transform_motor()
    #motor_control.extend_wheels_safe()
    #time.sleep(1)
    motor_control.move_forward_time(5)
    #motor_control.retract_wheels_safe()
    
    

    
if __name__ == "__main__":
    ultrasonic_pins = {
        'front': {'trig': 35, 'echo': 36},
        'left': {'trig': 29, 'echo': 31},
        'right': {'trig': 32, 'echo': 33}
    }

    camera_params = {
        'dispW': 1280,
        'dispH': 720
    }
    hsv_params = {
        'G': [45, 87, 49, 255, 95, 255],
        'R': [114, 179, 53, 255, 94, 246]
    }

    sensor_control = SensorControl(ultrasonic_pins, camera_params, hsv_params)
    motor_control = MotorControl(
        left_motor_pins={'in1': 12, 'in2': 11},
        right_motor_pins={'in1': 13, 'in2': 15},
        transform_motor_pins={'in1': 16, 'in2': 18}
    )
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(37, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    try:
        while True:
            state = GPIO.input(37)
            if state == GPIO.LOW:
                run_test()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("111")
    finally:
        GPIO.cleanup()


        #motor_control.stop()
        #motor_control.stop_transform_motor()
        #motor_control.extend_wheels_safe()
        #print("1")
        #time.sleep(5)
        #motor_control.retract_wheels_safe()
        #motor_control.move_forward()
        #time.sleep(1)
        #motor_control.stop()
        #time.sleep(1)
        #motor_control.move_backward()
        #time.sleep(1)
        #motor_control.stop()
        #time.sleep(1)
        #motor_control.turn_left()
        #time.sleep(1)
        #motor_control.stop()
        #time.sleep(1)
        #motor_control.turn_right()
        #time.sleep(1)
        #motor_control.stop()
        #time.sleep(1)
        #print("1")
        #motor_control.cleanup()
    #except KeyboardInterrupt:
        #motor_control.cleanup()