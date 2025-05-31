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
            ScanData object with distances_mm and angles_deg attributes
        """
        # Get the range image from the lidar
        range_image = self.lidar.getRangeImage()
        
        # Check if we have only one point or very few points
        if len(range_image) <= 1:
            print(f"[INFO] Generating 360 points from {len(range_image)} LiDAR point(s)")
            
            # Use the single distance value or default to max range
            distance = range_image[0] if range_image else self.max_range
            
            # Create synthetic scan with 360 points
            distances_mm = []
            angles_deg = []
            
            # Generate points around a full circle
            for angle in range(0, 360):
                # Could add some noise here for realism
                distances_mm.append(distance * 1000.0)  # Convert to mm
                angles_deg.append(angle - 180)  # Range from -180 to 179
            
            # Create and return scan data object
            class ScanData:
                def __init__(self):
                    self.distances_mm = distances_mm
                    self.angles_deg = angles_deg
                    
            return ScanData()
        
        # If we have multiple points, process normally
        # Get point cloud from LiDAR
        point_cloud = self.lidar.getPointCloud()
        
        # Initialize scan data
        distances_mm = []
        angles_deg = []
        
        if not point_cloud:
            # No point cloud data available yet
            # Generate default scan with max range
            for i in range(self.horizontal_resolution):
                angle = (i / self.horizontal_resolution) * 2 * math.pi - math.pi
                distances_mm.append(self.max_range * 1000.0)
                angles_deg.append(angle * 180.0 / math.pi)  # Convert to degrees
                
        else:
            # Process each horizontal point in the middle layer
            layer_index = self.num_layers // 2  # Use middle layer
            
            for i in range(self.horizontal_resolution):
                point_index = layer_index * self.horizontal_resolution + i
                
                # Calculate angle
                angle = (i / self.horizontal_resolution) * 2 * math.pi - math.pi
                angles_deg.append(angle * 180.0 / math.pi)  # Convert to degrees
                
                if point_index < len(point_cloud):
                    point = point_cloud[point_index]
                    # Calculate distance using Euclidean distance
                    distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
                    
                    # Convert to mm for compatibility with SLAM
                    distances_mm.append(distance * 1000.0)
                else:
                    # If point is not available, use max range
                    distances_mm.append(self.max_range * 1000.0)
        
        # Create and return scan data object
        class ScanData:
            def __init__(self):
                self.distances_mm = distances_mm
                self.angles_deg = angles_deg
                
        return ScanData()

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