import RPi.GPIO as GPIO
import time
import smbus2

IN1 = 17  # gpio pin for in1
IN2 = 18  # gpio pin for in2
I2C_ADDRESS = 0x76  # bar02 sensor i2c address

# commands for the ms5837 sensor (pressure)
CMD_RESET = 0x1E
CMD_ADC_READ = 0x00
CMD_PRESSURE_CONV = 0x40
CMD_TEMPERATURE_CONV = 0x50

# initialize i2c bus
bus = smbus2.SMBus(1)

# initialize gpio
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# function to reset sensor
def reset_sensor():
    bus.write_byte(I2C_ADDRESS, CMD_RESET)
    time.sleep(0.5)  # Increased wait time after reset to allow sensor to initialize

# read calibration data from prom
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
            prom.append(0)  # Append 0 if there is an error reading PROM data
    return prom

# read adc values
def read_adc(cmd):
    try:
        bus.write_byte(I2C_ADDRESS, cmd)
        time.sleep(0.01)  # Wait for conversion to complete
        adc_bytes = bus.read_i2c_block_data(I2C_ADDRESS, CMD_ADC_READ, 3)
        adc_value = (adc_bytes[0] << 16) | (adc_bytes[1] << 8) | adc_bytes[2]
        #print(f"Raw ADC Value: {adc_value}")
        return adc_value
    except IOError as e:
        print(f"I2C communication error: {e}")
        reset_i2c()  # Reset I2C and try again
        return 0

# function to reset i2c communication in case of issues
def reset_i2c():
    global bus
    bus.close()
    time.sleep(0.1)
    bus = smbus2.SMBus(1)

# calculate pressure and temperature from the sensor data
def calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature):
    # Extract calibration coefficients
    C1 = prom[1]
    C2 = prom[2]
    C3 = prom[3]
    C4 = prom[4]
    C5 = prom[5]
    C6 = prom[6]

    # Calculate temperature
    dT = adc_temperature - (C5 << 8)
    TEMP = 2000 + (dT * C6 >> 23)

    # Calculate pressure
    OFF = (C2 << 17) + ((C4 * dT) >> 6)
    SENS = (C1 << 16) + ((C3 * dT) >> 7)
    P = ((adc_pressure * SENS >> 21) - OFF) >> 15

    # Convert to kPa (1 kPa = 10 mbar)
    pressure_kpa = P / 1000.0
    temperature_c = TEMP / 100.0

    return pressure_kpa, temperature_c

# function to run pump forward (uses gpio 17)
def pump_forward():
    print("\n>>> Running pump forward\n")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

# function to run pump backward (uses gpio 18)
def pump_backward():
    print("\n<<< Running pump backward\n")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

# function to stop pump
def pump_stop():
    print("\n||| Stopping pump\n")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

reset_sensor()
prom = read_prom()

def read_print(iterations):
    adc_pressure = 0 
    adc_temperature = 0  
    for i in range(iterations):
        adc_pressure = read_adc(CMD_PRESSURE_CONV)
        adc_temperature = read_adc(CMD_TEMPERATURE_CONV)
        
        if adc_pressure != 0 and adc_temperature != 0:
            pressure_kpa, temperature_c = calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature)

            print(f"Pressure: {pressure_kpa:.2f} kPa")
            print(f"Temperature: {temperature_c:.2f} °C")
        
        time.sleep(1) 



try:
    while True:
        # Run pump forward
        pump_forward()
        read_print(5)

        # Stop pump
        pump_stop()
        read_print(1)
        
        # Run pump backward
        pump_backward()
        read_print(5)
        
        # Stop pump
        pump_stop()
        read_print(1)

except KeyboardInterrupt:
    GPIO.cleanup()
    print("Program exited")

finally:
    bus.close()
