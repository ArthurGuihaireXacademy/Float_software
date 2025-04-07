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

# Initialize hardware
bus = smbus2.SMBus(1)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# Initialize depth hold controller
DEPTH_HOLD_TIME_STEP = 0.1
depth_controller = DepthController(Kp=0.5, Ki=0.1, Kd=0.05, dt=DEPTH_HOLD_TIME_STEP)

def depth_hold(target_depth = 1.5, hold_time_seconds=3.5, log_file = None):
    depth_controller.start_depth_hold(target_depth * 98.1 + 1013) # Conversion meters underwater to milibar
    for i in range(int(hold_time_seconds / DEPTH_HOLD_TIME_STEP)):
        adc_pressure = read_adc(CMD_PRESSURE_CONV)
        adc_temperature = read_adc(CMD_TEMPERATURE_CONV)
        if adc_pressure and adc_temperature:
            pressure, temp = calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature)
            if i%int(1/DEPTH_HOLD_TIME_STEP) == 0:
                log_data(pressure, temp, log_file)
            output = depth_controller.update_depth_hold(pressure)
            if output > 2.5:
                pump_backward()
            elif output < -2.5:
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
            print(f"PROM[{i}] = {word}")
        except Exception as e:
            print(f"Error reading PROM[{i}]: {e}")
            prom.append(0)
    return prom

def read_adc(cmd):
    try:
        bus.write_byte(I2C_ADDRESS, cmd)
        time.sleep(0.01)
        adc_bytes = bus.read_i2c_block_data(I2C_ADDRESS, CMD_ADC_READ, 3)
        return (adc_bytes[0] << 16) | (adc_bytes[1] << 8) | adc_bytes[2]
    except IOError as e:
        print(f"I2C communication error: {e}")
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

def pump_forward():
    print("\n>>> Running pump forward\n")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def pump_backward():
    print("\n<<< Running pump backward\n")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def pump_stop():
    print("\n||| Stopping pump\n")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def read_print(iterations, log_file=None):
    for _ in range(iterations):
        adc_pressure = read_adc(CMD_PRESSURE_CONV)
        adc_temperature = read_adc(CMD_TEMPERATURE_CONV)
        
        if adc_pressure and adc_temperature:
            pressure, temp = calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature)
            log_data(pressure, temp, log_file)

        time.sleep(1)

def log_data(pressure, temp, log_file = None):
    timestamp = datetime.now().isoformat()
    print(f"[{timestamp}] Pressure: {pressure:.2f} kPa")
    print(f"[{timestamp}] Temperature: {temp:.2f} °C")
            
    # Write to file if log_file provided
    if log_file:
        log_file.write(f"{timestamp}, {pressure:.2f}, {temp:.2f}\n")
        log_file.flush()

# Initialize sensor
reset_sensor()
prom = read_prom()

# Main program
try:
    with open('/home/beans/sensor_data.txt', 'a') as log_file:
        # Write header if file is empty
        if os.path.getsize('/home/beans/sensor_data.txt') == 0:
            log_file.write("timestamp, pressure_kpa, temperature_c\n")
        
        while True:
            pump_forward()
            read_print(5, log_file)

            depth_hold(1.5, 5.0, log_file)
            read_print(1, log_file)
            
            pump_backward()
            read_print(5, log_file)
            
            pump_stop()
            read_print(1, log_file)

except KeyboardInterrupt:
    print("\nProgram stopped by user")
except Exception as e:
    print(f"\nError occurred: {e}")
finally:
    GPIO.cleanup()
    bus.close()
    print("Hardware resources released")