"""
Adapter for Webots IMU sensor
Bridges between Webots IMU API and our software's IMU interface
"""

import numpy as np
import math

class WebotsIMUAdapter:
    def __init__(self, imu_device, accelerometer_device, gyro_device, config):
        """
        Initialize the IMU adapter
        
        Args:
            imu_device: The Webots InertialUnit device
            accelerometer_device: The Webots Accelerometer device
            gyro_device: The Webots Gyro device
            config: Configuration parameters
        """
        self.imu = imu_device
        self.accelerometer = accelerometer_device
        self.gyro = gyro_device
        self.config = config
        
        # Noise parameters for simulation realism
        self.accel_noise_std = config.ACCEL_NOISE_STD
        self.gyro_noise_std = config.GYRO_NOISE_STD
        
        print("[INFO] IMU adapter initialized")

    def get_data(self):
        """
        Get the current IMU data
        
        Returns:
            Dictionary with accelerometer, gyroscope, and orientation data
        """
        # Get raw sensor data
        accel_raw = self.accelerometer.getValues()
        gyro_raw = self.gyro.getValues()
        orientation_raw = self.imu.getRollPitchYaw()
        
        # Add some noise to simulate real sensor behavior
        accel = np.array(accel_raw) + np.random.normal(0, self.accel_noise_std, 3)
        gyro = np.array(gyro_raw) + np.random.normal(0, self.gyro_noise_std, 3)
        
        # Package data
        imu_data = {
            'accel': accel,
            'gyro': gyro,
            'orientation': orientation_raw,
            'roll': orientation_raw[0],
            'pitch': orientation_raw[1],
            'yaw': orientation_raw[2]
        }
        
        return imu_data

    def get_accel_data(self):
        """
        Get accelerometer data only
        
        Returns:
            Accelerometer data (x, y, z)
        """
        accel_raw = self.accelerometer.getValues()
        accel = np.array(accel_raw) + np.random.normal(0, self.accel_noise_std, 3)
        return accel

    def get_gyro_data(self):
        """
        Get gyroscope data only
        
        Returns:
            Gyroscope data (x, y, z)
        """
        gyro_raw = self.gyro.getValues()
        gyro = np.array(gyro_raw) + np.random.normal(0, self.gyro_noise_std, 3)
        return gyro

    def get_orientation(self):
        """
        Get orientation data only
        
        Returns:
            Orientation data (roll, pitch, yaw)
        """
        return self.imu.getRollPitchYaw()