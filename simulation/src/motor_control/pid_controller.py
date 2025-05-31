# simulation/src/motor_control/pid_controller.py
class PID:
    """PID controller for motor control"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        """Initialize PID controller with gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        
    def compute(self, setpoint, feedback, dt=0.01):
        """Compute control output"""
        # Calculate error
        error = setpoint - feedback
        
        # Calculate derivative
        derivative = (error - self.prev_error) / dt
        
        # Update integral
        self.integral += error * dt
        
        # Calculate output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Store error for next iteration
        self.prev_error = error
        
        return output
        
    def reset(self):
        """Reset controller state"""
        self.prev_error = 0.0
        self.integral = 0.0