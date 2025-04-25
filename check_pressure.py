import ms5837
import time

ext_pressure_sensor = ms5837.MS5837_02BA(bus=1)
ext_pressure_sensor.init()

while True:
    ext_pressure_sensor.read()
    print(f"{ext_pressure_sensor.pressure():.5f}mbar")
    time.sleep(0.5)