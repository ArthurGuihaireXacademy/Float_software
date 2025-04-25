import depth_hold
import time
try:
    import RPi.GPIO as GPIO
except:
    print('GPIO not found, disabling motors')
    import disable_GPIO as GPIO
import threading
import csv
from datetime import datetime
import ms5837
import socket_client
import random

# === Client Socket Setup ===
TOPSIDE_SERVER_IP = '192.168.1.160'  # Replace with your computer's IP
TOPSIDE_SERVER_PORT = 8099
TEAM_CODE = 'FLOAT-TEAM-001'

# === Motor Pins ===
IN1 = 17
IN2 = 18
target_depth = 0.4
current_depth = 0
# === Logging ===
pressure_log_running = True
pressure_log_file = "pressure_data.csv"

# === GPIO Setup ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# === Pressure Sensor Init ===
ext_pressure_sensor = ms5837.MS5837_02BA(bus=1)
ext_pressure_sensor.init()
initial_pressure_offset = 0

# === Logging and Time ===
def get_timestamp():
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S')

def log_message(message, send_to_topside):
    timestamp = get_timestamp()
    print(f"[{timestamp}] {message}")
    if send_to_topside:
        socket_client.add_packet(f"[{timestamp}] {message}")

# === Sensor Calibration ===
def calibrate_sensor():
    global initial_pressure_offset
    log_message("Calibrating surface pressure...", False)
    pressures = []
    for _ in range(10):
        ext_pressure_sensor.read()
        pressures.append(ext_pressure_sensor.pressure())
        time.sleep(0.2)
    if pressures:
        initial_pressure_offset = sum(pressures) / len(pressures)
        log_message(f"Calibrated surface pressure: {initial_pressure_offset:.2f} mbar", False)

# === Depth Calculation ===
def update_depth():
    global current_depth
    if ext_pressure_sensor.read():
        current_pressure = ext_pressure_sensor.pressure()  # in mbar
        current_depth = max(0.0, (current_pressure - initial_pressure_offset) / 98.1)  # mbar to m
        return True
    else:
        current_depth = random.random()
        return False

# === Pressure Logger Thread ===
def pressure_logger():
    global pressure_log_running
    with open(pressure_log_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "Depth_meters"])

    while pressure_log_running:
        try:
            timestamp = get_timestamp()
            log_message(f"Logged Depth: {current_depth:.3f} m", True)
            with open(pressure_log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, f"{current_depth:.3f}"])
        except Exception as e:
            log_message(f"Error logging depth: {e}", False)
        time.sleep(1)

# === Motor Control ===
def descend(duration=9):
    log_message("Descending...", False)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    log_message("Descent complete", False)

def ascend(duration=15):
    log_message("Ascending...", False)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    log_message("Ascent complete", False)

def hold_depth(y, target_depth):
    log_message(f"Starting depth hold at {target_depth} meters...", False)

    PID_holder = depth_hold.PID_controller(6, 0, -40)
    PID_holder.start_depth_hold(y, target_depth)

    dt = 0.2  # control loop interval (s)
    hold_duration = 60 # seconds to hold depth
    elapsed = 0

    try:
        while elapsed < hold_duration:
            update_depth()
            pid_output = PID_holder.update_depth_hold(current_depth)

            log_message(f"[Depth Hold] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)

            if pid_output > 0.05:  # Too shallow ? descend
                log_message(f"[Depth Decends] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)

            elif pid_output < -0.05:  # Too deep ? ascend
                log_message(f"[Depth Acending] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)
            else:  # Within acceptable range ? hold
                log_message(f"[Depth Hold] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.LOW)

            time.sleep(dt)
            elapsed += dt
            print(elapsed)

        log_message("Depth hold complete.", False)
    finally:
        # Stop the motors when done
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        PID_holder.stop_depth_hold()

# === Socket Comms ===
def send_packets():
    socket_client.connect(TOPSIDE_SERVER_IP)
    socket_client.send_packets()
    socket_client.disconnect()

def add_packet(packet):
    socket_client.add_packet(packet)

# === Main Sequence ===
def main_sequence():
    global pressure_log_running
    for i in range(1):
        log_message("Float waiting at surface...", False)

        calibrate_sensor()
        update_depth()
        add_packet(f"CONNECTED\nTime: {get_timestamp()}\nTeam: {TEAM_CODE}\nDepth: {current_depth:.2f} m\n")
        send_packets()
        input('[Press ENTER to start]')

        logger_thread = threading.Thread(target=pressure_logger)
        logger_thread.start()

        #descend(20)
        #time.sleep(1)
        while not update_depth():
            print("Depth hold not started")
            time.sleep(0.5)
        hold_depth(current_depth, target_depth) # Optional: add your depth hold logic her
        ascend()
        time.sleep(1)

        log_message("Profile sequence complete.", False)
        send_packets()

    pressure_log_running = False
    logger_thread.join()
    add_packet("done")
    send_packets()
    GPIO.cleanup()
    log_message("Shutdown complete.", False)

if __name__ == "__main__":
    main_sequence()

