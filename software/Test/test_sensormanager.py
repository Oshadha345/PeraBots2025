# software/test/test_sensor_manager.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sensors.sensor_manager import SensorManager

sm = SensorManager()

for i in range(3):
    data = sm.read_all()
    print(f"\nRead #{i+1}")
    for k, v in data.items():
        print(f"{k}: {v}")
