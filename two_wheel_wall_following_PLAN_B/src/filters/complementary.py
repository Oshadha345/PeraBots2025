class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0

    def update(self, accel, gyro, dt):
        """Update the angle using accelerometer and gyroscope data."""
        # Integrate the gyroscope data to get the angle
        gyro_angle = self.angle + gyro[2] * dt
        
        # Calculate the angle from the accelerometer
        accel_angle = math.atan2(accel[1], accel[0])
        
        # Apply the complementary filter
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        
        return self.angle