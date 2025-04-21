import depth_hold
import numpy as np
import matplotlib.pyplot as plt
import random

import socket
import time
import RPi.GPIO as GPIO
import threading
import csv
import os
from datetime import datetime
import ms5837
import socket_client

# === Client Socket Setup ===
TOPSIDE_SERVER_IP = '192.168.1.160'  # Replace with your computer's IP
TOPSIDE_SERVER_PORT = 8099
TEAM_CODE = 'FLOAT-TEAM-001'

client_socket = None

# === Motor Pins ===
IN1 = 17
IN2 = 18
target_depth = 1035
y = 1013.25
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
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def log_message(message):
    timestamp = get_timestamp()
    print(f"[{timestamp}] {message}")
    socket_client.add_packet(f"[{timestamp}] {message}")

# === Sensor Calibration ===
def calibrate_sensor():
    global initial_pressure_offset
    log_message("Calibrating surface pressure...")
    pressures = []
    for _ in range(10):
        ext_pressure_sensor.read()
        pressures.append(ext_pressure_sensor.pressure())
        time.sleep(0.2)
    if pressures:
        initial_pressure_offset = sum(pressures) / len(pressures)
        log_message(f"Calibrated surface pressure: {initial_pressure_offset:.2f} mbar")

# === Depth Calculation ===
def get_depth():
    ext_pressure_sensor.read()
    time.sleep(1)
    current_pressure = ext_pressure_sensor.pressure()  # in mbar
    depth_m = max(0.0, (current_pressure - initial_pressure_offset) / 98.1)  # mbar to m
    return current_pressure

# === Pressure Logger Thread ===
def pressure_logger():
    global pressure_log_running
    if not os.path.exists(pressure_log_file):
        with open(pressure_log_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Depth_meters"])

    while pressure_log_running:
        try:
            depth = get_depth()
            timestamp = get_timestamp()
            log_message(f"Logged Depth: {depth:.3f} m")
            with open(pressure_log_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, f"{depth:.3f}"])
                socket_client.add_packet(f"Time: {timestamp}, Depth: {depth:.3f}")
        except Exception as e:
            log_message(f"Error logging depth: {e}")
        time.sleep(1)

# === Motor Control ===
def descend(duration=9):
    log_message("Descending...")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(duration)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    log_message("Descent complete")

def ascend(duration=15):
    log_message("Ascending...")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    log_message("Ascent complete")
def hold_depth(y, target_depth):
    log_message(f"Starting depth hold at {target_depth} meters...")

    PID_holder = depth_hold.PID_controller()
    PID_holder.start_depth_hold(y, target_depth)

    dt = 1  # control loop interval (s)
    hold_duration = 20 # seconds to hold depth
    elapsed = 0

    try:
        while elapsed < hold_duration:
            current_depth = get_depth()
            time.sleep(1)
            pid_output = PID_holder.update_depth_hold(current_depth)

            log_message(f"[Depth Hold] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}")

            if pid_output > 0.5:  # Too shallow ? descend
                log_message(f"[Depth Decends] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}")
                GPIO.output(IN1, GPIO.HIGH)
                GPIO.output(IN2, GPIO.LOW)
                
            elif pid_output < -0.5:  # Too deep ? ascend
                log_message(f"[Depth Acending] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}")
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.HIGH)
            else:  # Within acceptable range ? hold
                log_message(f"[Depth Hold] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}")
                GPIO.output(IN1, GPIO.LOW)
                GPIO.output(IN2, GPIO.LOW)

            time.sleep(dt)
            elapsed += dt
            print(elapsed)

        log_message("Depth hold complete.")
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
        log_message("Float waiting at surface...")

        calibrate_sensor()

        add_packet(f"CONNECTED\nTime: {get_timestamp()}\nTeam: {TEAM_CODE}\nDepth: {get_depth():.2f} m\n")
        send_packets()

        logger_thread = threading.Thread(target=pressure_logger)
        logger_thread.start()

        descend()
        time.sleep(2)

        hold_depth(y,target_depth) # Optional: add your depth hold logic here
        print("held")
        ascend()
        time.sleep(2)

        log_message("Profile sequence complete.")
        send_packets()

    
    pressure_log_running = False
    if client_socket:
        client_socket.close()
    logger_thread.join()
    GPIO.cleanup()
    log_message("Shutdown complete.")

if __name__ == "__main__":
    main_sequence()
