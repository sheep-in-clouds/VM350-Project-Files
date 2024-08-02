import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        state = GPIO.input(37)
        print(state)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("111")
finally:
    GPIO.cleanup()