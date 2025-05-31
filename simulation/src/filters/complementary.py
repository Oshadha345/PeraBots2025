# simulation/src/filters/complementary.py
import numpy as np

class ComplementaryFilter:
    """Simple complementary filter for orientation estimation"""
    
    def __init__(self, alpha=0.98):
        """Initialize filter with weight parameter"""
        self.alpha = alpha
        self.orientation = np.zeros(3)
        
    def update(self, accel, gyro, direct_orientation, dt=0.01):
        """Update orientation estimate using sensor fusion"""
        # Use gyro for short-term changes
        gyro_orientation = self.orientation + gyro * dt
        
        # Use accelerometer for long-term reference
        if np.linalg.norm(accel) > 0:
            accel_norm = accel / np.linalg.norm(accel)
            accel_orientation = np.array([
                np.arctan2(accel_norm[1], accel_norm[2]),
                np.arctan2(-accel_norm[0], np.sqrt(accel_norm[1]**2 + accel_norm[2]**2)),
                0  # Can't determine yaw from accelerometer
            ])
            
            # Combine with direct orientation for yaw
            accel_orientation[2] = direct_orientation[2]
            
            # Apply complementary filter
            self.orientation = self.alpha * gyro_orientation + (1 - self.alpha) * accel_orientation
        else:
            # If accelerometer is unreliable, use gyro only
            self.orientation = gyro_orientation
            
        return self.orientation