# sensors/sensor_manager.py

from platform import system

IS_REAL = system() != "Windows"

if IS_REAL:
    from .imu import MPU6050 as IMU
    from .encoder import Encoder
    from .lidar import XiaomiLidar as Lidar
else:
    from .mock.imu_mock import MockIMU as IMU
    from .mock.encoder_mock import MockEncoder as Encoder
    from .mock.lidar_mock import MockLidar as Lidar

class SensorManager:
    def __init__(self):
        self.imu = IMU()
        self.encoder = Encoder()
        self.lidar = Lidar()

    def read_all(self):
        return {
            "accel": self.imu.get_accel_data(),
            "gyro": self.imu.get_gyro_data(),
            "odometry": self.encoder.get_ticks(),
            "scan": self.lidar.get_scan()
        }
