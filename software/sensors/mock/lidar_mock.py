# sensors/mock/lidar_mock.py

import numpy as np

class MockLidar:
    def get_scan(self):
        # Simulated sinusoidal walls for 360°
        return [1000 + 150 * np.sin(i / 15.0) for i in range(360)]

