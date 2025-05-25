# software/test/test_lidar.py

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sensors import MockLidar

lidar = MockLidar()

scan = lidar.get_scan()

print("LIDAR scan sample (first 10 degrees):")
print(scan[:10])
print(f"Total points: {len(scan)}")
