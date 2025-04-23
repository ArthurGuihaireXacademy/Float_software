import depth_hold
import numpy as np
import matplotlib.pyplot as plt
import random

import socket
import time
try:
    import RPi.GPIO as GPIO
except:
    print("GPIO not found, disabling pump")
    import disable_GPIO as GPIO
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

# === Motor Pins ===
IN1 = 17
IN2 = 18

# === Logging ===
pressure_log_file = "pressure_data.csv"

# === GPIO Setup ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# === Pressure Sensor Init ===
try:
    ext_pressure_sensor = ms5837.MS5837_02BA(bus=1)
    ext_pressure_sensor.init()
except Exception as e:
    print(f"Error initializing pressure sensor: {e}")
initial_pressure_offset = 0

class FloatController():
    def __init__(self, depth_hold_depth = 0.4):
        self.pressure_log_running = True
        self.PID_holder = depth_hold.PID_controller(15.0, 0.05, -2.0, 1)
        self.target_depth = depth_hold_depth

    # === Logging and Time ===
    def get_timestamp(self):
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def log_message(self, message, send_to_topside):
        timestamp = self.get_timestamp()
        print(f"[{timestamp}] {message}")
        if send_to_topside:
            socket_client.add_packet(f"[{timestamp}] {message}")

    # === Sensor Calibration ===
    def calibrate_sensor(self):
        global initial_pressure_offset
        self.log_message("Calibrating surface pressure...", False)
        pressures = []
        for _ in range(10):
            ext_pressure_sensor.read()
            pressures.append(ext_pressure_sensor.pressure())
            time.sleep(0.2)
        if pressures:
            initial_pressure_offset = sum(pressures) / len(pressures)
            self.log_message(f"Calibrated surface pressure: {initial_pressure_offset:.2f} mbar", False)

    # === Depth Calculation ===
    def get_depth(self):
        ext_pressure_sensor.read()
        current_pressure = ext_pressure_sensor.pressure()  # in mbar
        depth_m = max(0.0, (current_pressure - initial_pressure_offset) / 98.1)  # mbar to m
        return depth_m

    # === Pressure Logger Thread ===
    def pressure_logger(self):
        if not os.path.exists(pressure_log_file):
            with open(pressure_log_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Timestamp", "Depth_meters"])

        while self.pressure_log_running:
            try:
                depth = self.get_depth()
                timestamp = self.get_timestamp()
                self.log_message(f"Logged Depth: {depth:.3f} m", True)
                with open(pressure_log_file, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow([timestamp, f"{depth:.3f}"])
            except Exception as e:
                self.log_message(f"Error logging depth: {e}", False)
            time.sleep(1)

    # === Motor Control ===
    def descend(self, duration=9):
        self.log_message("Descending...", False)
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        time.sleep(duration)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        self.log_message("Descent complete", False)

    def ascend(self, duration=15):
        self.log_message("Ascending...", False)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        self.log_message("Ascent complete", False)

    def hold_depth(self, y):
        self.log_message(f"Starting depth hold at {self.target_depth} meters...", False)

        self.PID_holder.start_depth_hold(y, self.target_depth)

        dt = 1  # control loop interval (s)
        hold_duration = 60 # seconds to hold depth
        elapsed = 0

        try:
            while elapsed < hold_duration:
                current_depth = self.get_depth()
                pid_output = self.PID_holder.update_depth_hold(current_depth)
                self.log_message(f"[Depth Hold] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)
                if pid_output > 0.05:
                    self.log_message(f"[Depth Decends] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)
                    GPIO.output(IN1, GPIO.HIGH)
                    GPIO.output(IN2, GPIO.LOW)
                elif pid_output < -0.05:
                    self.log_message(f"[Depth Acending] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.HIGH)
                else:
                    self.log_message(f"[Depth Hold] Depth: {current_depth:.2f} m | PID: {pid_output:.2f}", False)
                    GPIO.output(IN1, GPIO.LOW)
                    GPIO.output(IN2, GPIO.LOW)
                time.sleep(dt)
                elapsed += dt
            self.log_message("Depth hold complete.", False)
        finally:
            # Stop the motors when done
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            self.PID_holder.stop_depth_hold()

    # === Socket Comms ===
    def send_packets(self):
        socket_client.connect(TOPSIDE_SERVER_IP)
        socket_client.send_packets()
        socket_client.disconnect()

    def add_packet(self, packet):
        socket_client.add_packet(packet)

    # === Main Sequence ===
    def main_sequence(self):
        for i in range(1):
            self.log_message("Float waiting at surface...", False)

            self.calibrate_sensor()
            self.add_packet(f"CONNECTED\nTime: {self.get_timestamp()}\nTeam: {TEAM_CODE}\nDepth: {self.get_depth():.2f} m\n")
            threading.Thread(target=self.send_packets).start()
            time.sleep(10)

            logger_thread = threading.Thread(target=self.pressure_logger)
            logger_thread.start()

            #descend(20)
            #time.sleep(1)
            error = True
            while error:
                try:
                    global depth
                    depth = self.get_depth()
                    error = False
                except:
                    error = True
                    time.sleep(1)
                    print("Depth hold not started")
            self.hold_depth(depth) # Optional: add your depth hold logic her
            print("held")
            self.ascend()
            time.sleep(1)

            self.log_message("Profile sequence complete.", False)
            self.add_packet("done")
            threading.Thread(target=self.send_packets).start()

        self.pressure_log_running = False
        logger_thread.join()
        GPIO.cleanup()
        self.log_message("Shutdown complete.", False)

if __name__ == "__main__":
    controller = FloatController()
    controller.main_sequence()

