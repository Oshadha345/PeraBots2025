import time
import math

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0
        self.last_time = time.time()
        
    def update(self, accel, gyro, orientation=None):
        """Updated filter that handles sequence gyro data"""
        # Extract time since last update
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        # Make sure gyro is a scalar value
        gyro_z = float(gyro[2]) if isinstance(gyro[2], (list, tuple)) else gyro[2]
        
        # Update angle using gyro
        gyro_angle = self.angle + gyro_z * dt
        
        # If we have accelerometer data, use it to correct
        if accel is not None:
            # Calculate tilt from accelerometer
            accel_angle = math.atan2(accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
            
            # Apply complementary filter
            self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        else:
            self.angle = gyro_angle
        
        return self.angle