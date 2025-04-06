import RPi.GPIO as GPIO
import time

# Define GPIO pins for H-Bridge control (using IN3 and IN4)
IN1 = 22  # GPIO pin forIN3
IN2 = 23  # GPIO pin for IN4

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Function to run the pump forward
def pump_forward():
    print("Running pump forward: IN1=HIGH, IN2=LOW")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    print(f"IN1 state: {GPIO.input(IN1)}, IN2 state: {GPIO.input(IN2)}")

# Function to run the pump backward
def pump_backward():
    print("Running pump backward: IN1=LOW, IN2=HIGH")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    print(f"IN1 state: {GPIO.input(IN1)}, IN2 state: {GPIO.input(IN2)}")

# Function to stop the pump
def pump_stop():
    print("Stopping pump: IN1=LOW, IN2=LOW")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    print(f"IN1 state: {GPIO.input(IN1)}, IN2 state: {GPIO.input(IN2)}")

try:
    while True:
        # Run pump forward for 5 seconds
        pump_forward()
        time.sleep(5)

        # Stop the pump for 2 seconds
        pump_stop()
        time.sleep(2)

        # Add a small delay before reversing
        time.sleep(0.5)

        # Run pump backward for 5 seconds
        pump_backward()
        time.sleep(5)

        # Stop the pump for 2 seconds
        pump_stop()
        time.sleep(2)

        # Add a small delay before reversing
        time.sleep(0.5)

except KeyboardInterrupt:
    # Clean up GPIO on exit
    GPIO.cleanup()
    print("Program exited cleanly")
