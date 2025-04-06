import RPi.GPIO as GPIO
import time


# Define GPIO pins
IN1 = 17  # L298N IN1
IN2 = 18  # L298N IN2
ENA = 22  # L298N ENA (PWM for speed control)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Initialize PWM for speed control
pwm = GPIO.PWM(ENA, 1000)  # 1kHz frequency
pwm.start(0)  # Start with pump off (0% speed)

def pump_backward(speed=100):
    """Move the peristaltic pump in reverse"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def pump_stop():
    """Stop the peristaltic pump"""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)  # Stop motor

# drain the pump
try:
    while True:
        print("Pumping Backward")
        pump_backward(80)
        time.sleep(10)
        

except KeyboardInterrupt:
    print("Stopping Pump")
    pump_stop()
finally:
    GPIO.cleanup()
