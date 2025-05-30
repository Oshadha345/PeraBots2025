class WebotsIMUAdapter:
    def __init__(self, imu, accelerometer, gyro, config):
        self.imu = imu
        self.accelerometer = accelerometer
        self.gyro = gyro
        self.config = config

        # Enable the sensors
        self.imu.enable(config.TIMESTEP)
        self.accelerometer.enable(config.TIMESTEP)
        self.gyro.enable(config.TIMESTEP)

    def get_data(self):
        """Retrieve IMU data including orientation, acceleration, and gyro readings."""
        orientation = self.imu.getRollPitchYaw()
        accel = self.accelerometer.getValues()
        gyro = self.gyro.getValues()

        return {
            'orientation': orientation,
            'accel': accel,
            'gyro': gyro
        }