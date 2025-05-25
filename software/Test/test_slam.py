import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from pera_slam import RPLidarA1, ParticleFilterSLAM, visualize_map

# Create a simulated LiDAR sensor
lidar = RPLidarA1(offset_mm=0)

# Create a SLAM instance
slam = ParticleFilterSLAM(
    laser=lidar,
    map_size_pixels=500,
    map_size_meters=10.0,
    map_quality=50,
    hole_width_mm=600,
    num_particles=100
)

# Simulate robot movement and scanning
for i in range(100):
    # Simulate robot movement (forward and turning slightly)
    dxy_mm = 50  # Move 50mm forward
    dtheta_degrees = 2  # Turn 2 degrees
    dt_seconds = 0.1  # 100ms between updates
    
    # Simulate a scan (replace with real sensor data)
    # This creates a simple circular room with an obstacle
    angles = np.linspace(0, 360, lidar.scan_size)
    distances = np.ones(lidar.scan_size) * 2000  # 2 meters to walls
    
    # Add simulated walls
    for j in range(len(distances)):
        angle_rad = np.radians(angles[j])
        # Add a box obstacle
        if 45 < angles[j] < 135 and i > 50:
            distances[j] = 1000  # Obstacle at 1 meter
    
    # Update SLAM
    slam.update(
        scans_mm=distances,
        pose_change=(dxy_mm, dtheta_degrees, dt_seconds),
        scan_angles_degrees=angles
    )
    
    # # Visualize the map every 10 steps
    # if i % 10 == 0:
    #     visualize_map(slam, with_particles=True)

# Final visualization
visualize_map(slam, with_particles=True)