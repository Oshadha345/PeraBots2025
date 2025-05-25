# software/test/test_imu.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sensors import MockIMU

imu = MockIMU()

for i in range(5):
    accel = imu.get_accel_data()
    gyro = imu.get_gyro_data()
    print(f"[{i+1}] Accelerometer: {accel}")
    print(f"[{i+1}] Gyroscope    : {gyro}")
    print("─" * 40)
