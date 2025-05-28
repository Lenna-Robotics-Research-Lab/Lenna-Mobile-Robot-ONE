import Jetson.GPIO as GPIO
import time
import rospy

# Use physical pin numbering
GPIO.setmode(GPIO.BOARD)

# Set pin 23 as output
GPIO.setup(19, GPIO.OUT)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        state = GPIO.input(21)

        if state == GPIO.HIGH:
            GPIO.output(19, GPIO.HIGH)  # LED ON
        else:
            GPIO.output(19, GPIO.LOW)   # LED OFF

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    GPIO.cleanup()  # Clean up GPIO settings

