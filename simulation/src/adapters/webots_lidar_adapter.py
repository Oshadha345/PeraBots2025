"""
Adapter for Webots LiDAR sensor
Bridges between Webots LiDAR API and our software's LiDAR interface
"""

import numpy as np
import math

class WebotsLidarAdapter:
    def __init__(self, lidar_device, config):
        """
        Initialize the LiDAR adapter
        
        Args:
            lidar_device: The Webots LiDAR device
            config: Configuration parameters
        """
        self.lidar = lidar_device
        self.config = config
        
        # Get LiDAR properties
        self.horizontal_resolution = self.lidar.getHorizontalResolution()
        self.num_layers = self.lidar.getNumberOfLayers()
        self.min_range = self.lidar.getMinRange()
        self.max_range = self.lidar.getMaxRange()
        self.fov = self.lidar.getFov()
        
        # Calculate angle for each point
        self.angles = np.linspace(
            -self.fov/2, 
            self.fov/2, 
            self.horizontal_resolution
        )
        
        print(f"[INFO] LiDAR initialized with {self.horizontal_resolution} points and FOV {self.fov} rad")

    def get_scan(self):
        """
        Get the current LiDAR scan
        
        Returns:
            List of distances in millimeters (for compatibility with SLAM)
        """
        # Get point cloud from LiDAR
        point_cloud = self.lidar.getPointCloud()
        
        # Convert to scan distances (use the middle layer)
        scan_mm = []
        
        if not point_cloud:
            # No point cloud data available yet
            return [self.max_range * 1000] * self.horizontal_resolution
        
        # Process each horizontal point in the middle layer
        layer_index = self.num_layers // 2  # Use middle layer
        
        for i in range(self.horizontal_resolution):
            point_index = layer_index * self.horizontal_resolution + i
            if point_index < len(point_cloud):
                point = point_cloud[point_index]
                # Calculate distance using Euclidean distance
                distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
                
                # Convert to mm for compatibility with SLAM
                scan_mm.append(distance * 1000.0)
            else:
                # If point is not available, use max range
                scan_mm.append(self.max_range * 1000.0)
        
        return scan_mm

    def get_angles(self):
        """
        Get the angles for each LiDAR point
        
        Returns:
            List of angles in degrees
        """
        return np.degrees(self.angles).tolist()

    def get_raw_data(self):
        """
        Get the raw point cloud data
        
        Returns:
            Raw point cloud data
        """
        return self.lidar.getPointCloud()

    def get_range_image(self):
        """
        Get the range image
        
        Returns:
            Range image data
        """
        return self.lidar.getRangeImage()