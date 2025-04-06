import RPi.GPIO as GPIO
import time

# Define GPIO pins for H-Bridge control
IN1 = 17  # GPIO pin for IN1
IN2 = 18  # GPIO pin for IN2

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Function to run the pump forward
def pump_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

# Function to run the pump backward
def pump_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

# Function to stop the pump
def pump_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

try:
    while True:
        # Run pump forward for 5 seconds
        pump_forward()
        print("Pump running forward")
        time.sleep(5)

        # Stop the pump for 2 seconds
        pump_stop()
        print("Pump stopped")
        time.sleep(2)

        # Run pump backward for 5 seconds
        pump_backward()
        print("Pump running backward")
        time.sleep(5)

        # Stop the pump for 2 seconds
        pump_stop()
        print("Pump stopped")
        time.sleep(2)

except KeyboardInterrupt:
    # Clean up GPIO on exit
    GPIO.cleanup()
    print("Program exited cleanly")
