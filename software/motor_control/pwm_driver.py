import time
from typing import Optional

# Try to import RPi.GPIO, but provide fallback for development on non-Pi systems
try:
    import RPi.GPIO as GPIO
    PI_AVAILABLE = True
except ImportError:
    PI_AVAILABLE = False
    print("Warning: RPi.GPIO not available, running in simulation mode")

class PWMDriver:
    """
    Low-level PWM driver for motor control.
    Interfaces with hardware-specific PWM channels.
    """
    
    def __init__(self, pin: int, direction_pin: Optional[int] = None, frequency: int = 20000):
        """
        Initialize the PWM driver.
        
        Args:
            pin: PWM output pin number
            direction_pin: Optional separate pin for direction control
            frequency: PWM frequency in Hz
        """
        self.pin = pin
        self.direction_pin = direction_pin
        self.frequency = frequency
        self.duty_cycle = 0
        self.direction = 1  # 1 for forward, -1 for reverse
        self.pwm = None  # Will hold the PWM instance
        
        # Initialize the PWM hardware
        self._initialize_hardware()
        
    def _initialize_hardware(self):
        """Initialize the hardware-specific PWM interface."""
        if not PI_AVAILABLE:
            print(f"[SIMULATION] Initialized PWM on pin {self.pin} at {self.frequency}Hz")
            return
            
        # Real hardware initialization for Raspberry Pi
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        
        # Create PWM instance
        self.pwm = GPIO.PWM(self.pin, self.frequency)
        self.pwm.start(0)  # Start with 0% duty cycle
        
        # Set up direction pin if provided
        if self.direction_pin is not None:
            GPIO.setup(self.direction_pin, GPIO.OUT)
            GPIO.output(self.direction_pin, GPIO.HIGH if self.direction > 0 else GPIO.LOW)
            
        print(f"Initialized PWM on pin {self.pin} at {self.frequency}Hz")
        
    def set_duty_cycle(self, duty_cycle: float, direction: int = 1):
        """
        Set the PWM duty cycle and direction.
        
        Args:
            duty_cycle: PWM duty cycle (0-100%)
            direction: Motor direction (1 for forward, -1 for reverse)
        """
        # Constrain duty cycle to valid range
        duty_cycle = max(min(duty_cycle, 100.0), 0.0)
        
        self.duty_cycle = duty_cycle
        self.direction = 1 if direction >= 0 else -1
        
        # Apply to hardware
        self._apply_to_hardware()
        
    def _apply_to_hardware(self):
        """Apply the current settings to the hardware."""
        if not PI_AVAILABLE:
            print(f"[SIMULATION] Setting PWM on pin {self.pin} to {self.duty_cycle}% " + 
                 f"(direction: {self.direction})")
            return
            
        if self.pwm is None:
            print("Warning: PWM not initialized")
            return
            
        # Apply duty cycle
        self.pwm.ChangeDutyCycle(self.duty_cycle)
        
        # Set direction if we have a direction pin
        if self.direction_pin is not None:
            GPIO.output(self.direction_pin, GPIO.HIGH if self.direction > 0 else GPIO.LOW)
        
    def stop(self):
        """Stop the PWM output."""
        self.set_duty_cycle(0)
        
    def cleanup(self):
        """Clean up GPIO resources."""
        if not PI_AVAILABLE:
            print("[SIMULATION] Cleaning up PWM resources")
            return
            
        if self.pwm is not None:
            self.pwm.stop()
        # Note: Don't call GPIO.cleanup() here as it might affect other pins
        # Let the application handle overall GPIO cleanup