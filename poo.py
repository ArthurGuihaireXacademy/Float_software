import RPi.GPIO as GPIO
import time
import smbus2
from datetime import datetime
import os
from depth_controller import DepthController

# Pin and sensor configuration
IN1 = 17  # gpio pin for in1
IN2 = 18  # gpio pin for in2
I2C_ADDRESS = 0x76  # bar02 sensor i2c address

# Sensor commands
CMD_RESET = 0x1E
CMD_ADC_READ = 0x00
CMD_PRESSURE_CONV = 0x40
CMD_TEMPERATURE_CONV = 0x50

# Constants
SURFACE_PRESSURE_KPA = 101.325  # Adjust based on local pressure
GRAVITY = 9.81  # m/s^2
HOVER_DEPTH = 2.5  # Depth in meters
HOVER_TIME = 45  # Hover time in seconds

# Initialize hardware
bus = smbus2.SMBus(1)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

DEPTH_HOLD_TIME_STEP = 0.2
depth_controller = DepthController(Kp=20.0, Ki=5.0, Kd=1.0, dt=DEPTH_HOLD_TIME_STEP)
depth_controller.load_calibration_file()

def depth_hold(target_depth = 2.5, hold_time_seconds=45):
    depth_controller.start_depth_hold(target_depth) # Conversion meters underwater to milibar
    for i in range(int(hold_time_seconds / DEPTH_HOLD_TIME_STEP)):
        pressure, temp, depth = read_pressure_and_depth()
        if depth:
            if i%int(1/DEPTH_HOLD_TIME_STEP) == 0:
                print(f"Depth: {depth:.2f}m")
            output = depth_controller.update_depth_hold(depth)
            if output > 1:
                pump_backward()
            elif output < -1:
                pump_forward()
            else:
                pump_stop()
        else:
            pump_stop()
        time.sleep(DEPTH_HOLD_TIME_STEP)
    depth_controller.stop_depth_hold()

def reset_sensor():
    bus.write_byte(I2C_ADDRESS, CMD_RESET)
    time.sleep(0.5)

def read_prom():
    prom = []
    for i in range(8):
        try:
            data = bus.read_i2c_block_data(I2C_ADDRESS, 0xA0 + (i * 2), 2)
            word = (data[0] << 8) | data[1]
            prom.append(word)
        except Exception as e:
            prom.append(0)
    return prom

def read_adc(cmd):
    try:
        bus.write_byte(I2C_ADDRESS, cmd)
        time.sleep(0.01)
        adc_bytes = bus.read_i2c_block_data(I2C_ADDRESS, CMD_ADC_READ, 3)
        return (adc_bytes[0] << 16) | (adc_bytes[1] << 8) | adc_bytes[2]
    except IOError:
        reset_i2c()
        return 0

def reset_i2c():
    global bus
    bus.close()
    time.sleep(0.1)
    bus = smbus2.SMBus(1)

def calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature):
    C1, C2, C3, C4, C5, C6 = prom[1:7]
    dT = adc_temperature - (C5 << 8)
    TEMP = 2000 + (dT * C6 >> 23)
    OFF = (C2 << 17) + ((C4 * dT) >> 6)
    SENS = (C1 << 16) + ((C3 * dT) >> 7)
    P = ((adc_pressure * SENS >> 21) - OFF) >> 15
    return P / 1000.0, TEMP / 100.0

def calculate_depth(pressure_kpa):
    return max(0, (pressure_kpa - SURFACE_PRESSURE_KPA) * 100 / GRAVITY)

def pump_forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def pump_backward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def pump_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def read_pressure_and_depth():
    adc_pressure = read_adc(CMD_PRESSURE_CONV)
    adc_temperature = read_adc(CMD_TEMPERATURE_CONV)
    
    if adc_pressure and adc_temperature:
        pressure, temp = calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature)
        depth = calculate_depth(pressure)
        return pressure, temp, depth
    return None, None, None

def reach_target_depth(target_depth):
    """Descend until reaching a specified depth."""
    while True:
        pressure, temp, depth = read_pressure_and_depth()
        print(f"Depth: {depth:.2f}m | Target: {target_depth}m")

        if depth >= target_depth:
            pump_stop()
            break
        pump_forward()
        time.sleep(1)

def ascend_to_surface():
    """Ascend until reaching the surface."""
    while True:
        pressure, temp, depth = read_pressure_and_depth()
        print(f"Depth: {depth:.2f}m | Target: Surface")

        if depth <= 0.2:  # Allow small buffer
            pump_stop()
            break
        pump_backward()
        time.sleep(1)

# Initialize sensor
reset_sensor()
prom = read_prom()

# Main program
try:
    with open('/home/beans/sensor_data.txt', 'a') as log_file:
        if os.path.getsize('/home/beans/sensor_data.txt') == 0:
            log_file.write("timestamp, pressure_kpa, temperature_c, depth_m\n")

        # Step 1: Descend to 2.5m
        print(">>> Descending to 2.5m")
        reach_target_depth(HOVER_DEPTH)
        
        # Step 2: Hover for 45 seconds
        print("||| Hovering at 2.5m for 45 seconds")
        #time.sleep(HOVER_TIME)
        depth_hold(target_depth=2.5, hold_time_seconds=45)

        # Step 3: Descend to bottom
        print(">>> Descending to bottom")
        reach_target_depth(3.65)  # Adjust based on pool depth

        # Step 4: Ascend to surface
        print("<<< Ascending to surface")
        ascend_to_surface()

        # Step 5: Descend again
        print(">>> Descending to bottom again")
        reach_target_depth(3.65)

        # Step 6: Ascend to surface again
        print("<<< Ascending to surface")
        ascend_to_surface()

except KeyboardInterrupt:
    print("\nProgram stopped by user")
except Exception as e:
    print(f"\nError occurred: {e}")
finally:
    GPIO.cleanup()
    bus.close()
    print("Hardware resources released")