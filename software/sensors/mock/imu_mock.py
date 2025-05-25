# sensors/mock/imu_mock.py

import random

class MockIMU:
    def get_accel_data(self):
        return {"x": 0.0, "y": 0.0, "z": 9.8}

    def get_gyro_data(self):
        return {
            "x": random.uniform(-0.01, 0.01),
            "y": 0.0,
            "z": random.uniform(0.04, 0.06)
        }
