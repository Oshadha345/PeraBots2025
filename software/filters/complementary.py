# filters/complementary_filter.py

import numpy as np
import time
import math

class ComplementaryFilter:
    """
    Complementary filter for IMU sensor fusion
    Combines accelerometer and gyroscope data to estimate orientation
    """
    
    def __init__(self, alpha=0.98):
        """
        Initialize the filter
        
        Args:
            alpha: Weight for gyroscope data (0.0 to 1.0)
        """
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        
    def update(self, accel, gyro, orientation=None):
        """
        Update the filter with new sensor data
        
        Args:
            accel: Accelerometer data [ax, ay, az]
            gyro: Gyroscope data [gx, gy, gz] in rad/s
            orientation: Current orientation from IMU (optional)
            
        Returns:
            [roll, pitch, yaw] filtered orientation in radians
        """
        # Extract time since last update
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        # Make sure we have numpy arrays
        accel = np.array(accel)
        gyro = np.array(gyro)
        
        # Extract gyro data - ensure they're scalar values
        gx = float(gyro[0])
        gy = float(gyro[1])
        gz = float(gyro[2])
        
        # Update angles using gyro
        self.pitch += gx * dt
        self.roll += gy * dt
        
        # If we have direct orientation from IMU, use it for yaw
        if orientation is not None:
            self.yaw = orientation[2]  # Assuming orientation is [roll, pitch, yaw]
        else:
            self.yaw += gz * dt
        
        # If accelerometer data is available, use it to correct roll and pitch
        if accel is not None and np.linalg.norm(accel) > 0:
            # Calculate roll and pitch from accelerometer
            accel_roll = math.atan2(accel[1], accel[2])
            accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
            
            # Complementary filter
            self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        
        return np.array([self.roll, self.pitch, self.yaw])