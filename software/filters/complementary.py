# filters/complementary_filter.py

import numpy as np

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        """
        Initialize the complementary filter.

        Parameters:
        - alpha: blending factor between gyro and accel (0 < alpha < 1)
        """
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0

    def update(self, accel, gyro, dt):
        """
        Update pitch and roll using accelerometer and gyroscope.

        Parameters:
        - accel: tuple/list/ndarray of (ax, ay, az)
        - gyro: tuple/list/ndarray of (gx, gy, gz), in degrees/sec
        - dt: time difference in seconds

        Returns:
        - (pitch, roll): estimated angles in degrees
        """
        ax, ay, az = accel
        gx, gy, _ = gyro

        # Integrate gyroscope to estimate angles
        self.pitch += gx * dt
        self.roll  += gy * dt

        # Calculate angle from accelerometer
        accel_pitch = np.arctan2(ay, np.sqrt(ax ** 2 + az ** 2)) * 180 / np.pi
        accel_roll  = np.arctan2(-ax, np.sqrt(ay ** 2 + az ** 2)) * 180 / np.pi

        # Blend the estimates
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        self.roll  = self.alpha * self.roll  + (1 - self.alpha) * accel_roll

        return self.pitch, self.roll
