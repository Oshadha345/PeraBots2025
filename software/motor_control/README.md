# 🔧 Motor Control

This module handles real-time control of motors using PWM and PID control algorithms.

## Files

- `pwm_driver.py`: Interface for Raspberry Pi PWM output (via GPIO or I2C motor driver)
- `pid_controller.py`: Generic PID class for speed/direction control

Make sure your motor drivers are properly connected to the Pi and configured via `core/config.py`.
