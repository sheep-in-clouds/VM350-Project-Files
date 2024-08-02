import RPi.GPIO as GPIO
from motor_control import MotorControl
from sensor_control import SensorControl
from task_manager import TaskManager
import time
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

# Initialize the SensorControl class
sensor_control = SensorControl(ultrasonic_pins, camera_params, hsv_params)

try:
    while True:
        # Read ultrasonic distances
        front_distance = sensor_control.read_front_distance()
        left_distance = sensor_control.read_left_distance()
        right_distance = sensor_control.read_right_distance()

        # Read IMU data
        pitch = sensor_control.read_imu_pitch()
        roll = sensor_control.read_imu_roll()

        # Read camera data
        camera_result = sensor_control.read_camera_data()

        # Output the sensor values
        print(f"Front Distance: {front_distance:.2f} cm")
        print(f"Left Distance: {left_distance:.2f} cm")
        print(f"Right Distance: {right_distance:.2f} cm")
        print(f"IMU Pitch: {pitch} degrees")
        print(f"IMU Roll: {roll} degrees")
        print(f"Camera Result: {camera_result}")

        # Sleep for a short period before the next loop
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by User")

finally:
    GPIO.cleanup()