from motor_control.pid_controller import PID
from motor_control.pwm_driver import PWMDriver

class MotorController:
    """
    High-level motor controller that converts desired speeds to PWM signals.
    """
    
    def __init__(self, left_pin: int, right_pin: int, encoder_left_pin: int = None, encoder_right_pin: int = None):
        """
        Initialize the motor controller.
        
        Args:
            left_pin: PWM pin for left motor
            right_pin: PWM pin for right motor
            encoder_left_pin: Optional encoder pin for left motor feedback
            encoder_right_pin: Optional encoder pin for right motor feedback
        """
        # Initialize PWM drivers
        self.left_pwm = PWMDriver(left_pin)
        self.right_pwm = PWMDriver(right_pin)
        
        # Initialize PID controllers if encoders are available
        self.use_pid = encoder_left_pin is not None and encoder_right_pin is not None
        if self.use_pid:
            self.left_pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)
            self.right_pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)
            
        # Initialize encoder pins (if provided)
        self.encoder_left_pin = encoder_left_pin
        self.encoder_right_pin = encoder_right_pin
        
        # Current speed values
        self.left_speed = 0.0
        self.right_speed = 0.0
    
    def set_motor_speeds(self, left_speed: float, right_speed: float):
        """
        Set the motor speeds.
        
        Args:
            left_speed: Normalized speed for left motor (-1.0 to 1.0)
            right_speed: Normalized speed for right motor (-1.0 to 1.0)
        """
        # Constrain to valid range
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)
        
        self.left_speed = left_speed
        self.right_speed = right_speed
        
        # Apply speeds directly if not using PID
        if not self.use_pid:
            self._apply_motor_speeds(left_speed, right_speed)
        
    def update(self, left_encoder_value: float = None, right_encoder_value: float = None):
        """
        Update the motor controller (used with PID).
        
        Args:
            left_encoder_value: Current encoder value for left motor
            right_encoder_value: Current encoder value for right motor
        """
        if not self.use_pid:
            return
            
        # Set PID setpoints based on desired speeds
        self.left_pid.setpoint = self.left_speed
        self.right_pid.setpoint = self.right_speed
        
        # Update PID controllers with encoder feedback
        left_output = self.left_pid.update(left_encoder_value)
        right_output = self.right_pid.update(right_encoder_value)
        
        # Apply the PID outputs to motors
        self._apply_motor_speeds(left_output, right_output)
        
    def _apply_motor_speeds(self, left_speed: float, right_speed: float):
        """
        Apply the motor speeds to the PWM drivers.
        
        Args:
            left_speed: Normalized speed for left motor (-1.0 to 1.0)
            right_speed: Normalized speed for right motor (-1.0 to 1.0)
        """
        # Convert normalized speeds to PWM duty cycles (usually 0-100%)
        left_pwm = abs(left_speed) * 100.0
        right_pwm = abs(right_speed) * 100.0
        
        # Set motor directions (may need to be implemented based on your hardware)
        left_direction = 1 if left_speed >= 0 else -1
        right_direction = 1 if right_speed >= 0 else -1
        
        # Apply PWM values
        self.left_pwm.set_duty_cycle(left_pwm, left_direction)
        self.right_pwm.set_duty_cycle(right_pwm, right_direction)
    
    def stop(self):
        """Stop both motors."""
        self.set_motor_speeds(0.0, 0.0)