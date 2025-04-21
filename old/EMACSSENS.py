import smbus2
import time

# Bar02 sensor I2C address (default is 0x76 or 0x77)
I2C_ADDRESS = 0x76

# Commands for the MS5837 sensor
CMD_RESET = 0x1E
CMD_ADC_READ = 0x00
CMD_PRESSURE_CONV = 0x40  # OSR=256
CMD_TEMPERATURE_CONV = 0x50  # OSR=256

# Initialize I2C bus
bus = smbus2.SMBus(1)

def reset_sensor():
    """Reset the sensor."""
    bus.write_byte(I2C_ADDRESS, CMD_RESET)
    time.sleep(0.01)  # Wait for reset to complete

def read_prom():
    """Read calibration data from PROM."""
    prom = []
    for i in range(7):
        # Read 2 bytes from PROM
        try:
            data = bus.read_i2c_block_data(I2C_ADDRESS, 0xA0 + (i * 2), 2)
            # Combine the two bytes into a single word
            word = (data[0] << 8) | data[1]
            prom.append(word)
            print(f"PROM[{i}] = {word}")
        except Exception as e:
            print(f"Error reading PROM[{i}]: {e}")
            prom.append(0)  # Append 0 if reading fails
    return prom

def read_adc(cmd):
    """Start a conversion and read the ADC result."""
    bus.write_byte(I2C_ADDRESS, cmd)
    time.sleep(0.01)  # Wait for conversion to complete
    adc_bytes = bus.read_i2c_block_data(I2C_ADDRESS, CMD_ADC_READ, 3)
    adc_value = (adc_bytes[0] << 16) | (adc_bytes[1] << 8) | adc_bytes[2]
    print(f"Raw ADC Value: {adc_value}")
    return adc_value

def calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature):
    """Calculate pressure and temperature using the calibration data."""
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

def main():
    # Reset the sensor
    reset_sensor()

    # Read calibration data from PROM
    prom = read_prom()

    try:
        while True:
            # Read pressure and temperature ADC values
            adc_pressure = read_adc(CMD_PRESSURE_CONV)
            adc_temperature = read_adc(CMD_TEMPERATURE_CONV)

            # Calculate pressure and temperature
            pressure_kpa, temperature_c = calculate_pressure_and_temperature(prom, adc_pressure, adc_temperature)

            # Print results
            print(f"Pressure: {pressure_kpa:.2f} kPa")
            print(f"Temperature: {temperature_c:.2f} °C")
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping sensor reading.")
    finally:
        bus.close()

if __name__ == "__main__":
    main()
