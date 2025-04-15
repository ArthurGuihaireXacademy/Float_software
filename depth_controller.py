from pathlib import Path

class DepthController:
    def __init__(self, Kp = 1.2, Ki = 0.15, Kd = 0.15, dt = 0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.calibrated_integral = 0
        self.calibrated = False
        self.holding = False

    def start_depth_hold(self, current_pressure, target_pressure=None):
        self.pressure = current_pressure
        if target_pressure is None:
            self.target_pressure = current_pressure
        else:
            self.target_pressure = target_pressure
        self.holding = True
        self.integral = self.calibrated_integral

    def stop_depth_hold(self):
        self.holding = False
        if not self.calibrated:
            self.calibrated_integral = self.integral
            self.calibrated = True

    def save_calibration_file(self):
        file_path = Path.home() / ".config" / "depth_hold_config"
        file_path.mkdir(parents=True, exist_ok=True)
        file_path /= "depth_hold_calibration.txt"
        with open(file_path, 'w') as config_file:
            config_file.write(self.calibrated_integral)

    def load_calibration_file(self):
        file_path = Path.home() / ".config" / "depth_hold_config" / "depth_hold_calibration.txt"
        if not file_path.exists():
            self.save_calibration_file()
            return False
        else:
            with open(file_path, 'r') as config_file:
                self.calibrated_integral = float(config_file.read())
            return True

    def update_depth_hold(self, current_pressure):
        if self.holding:
            derivative = (self.pressure - current_pressure) / self.dt
            self.pressure = current_pressure
            error = self.target_pressure - current_pressure
            self.integral += error * self.dt
            return self.dt * (self.Kp * error + self.Ki * self.integral - self.Kd * derivative)
        else:
            return 0