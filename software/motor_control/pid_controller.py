# pid_controller.py
import time

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, measured_value):
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.01

        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.last_error = error
        self.last_time = current_time
        return output

    def reset(self):
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()
