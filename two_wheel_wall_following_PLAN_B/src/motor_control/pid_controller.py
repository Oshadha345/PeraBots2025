class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, measured_value, dt):
        error = self.setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.previous_error = error
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.integral = 0.0  # Reset integral term on setpoint change
        self.previous_error = 0.0  # Reset previous error on setpoint change