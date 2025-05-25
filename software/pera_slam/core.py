import numpy as np
from abc import ABC, abstractmethod

class Position:
    """Position (pose) of the robot: x, y, theta"""
    
    def __init__(self, x_mm, y_mm, theta_degrees):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.theta_degrees = theta_degrees
    
    def copy(self):
        return Position(self.x_mm, self.y_mm, self.theta_degrees)
    
    def __str__(self):
        return f"Position: x = {self.x_mm:7.0f} mm  y = {self.y_mm:7.0f} mm  theta = {self.theta_degrees:+04.0f} degrees"

class Scan:
    """Represents a single LiDAR scan"""
    
    def __init__(self, laser, span=1):
        """
        Initialize a scan with a laser object
        laser: A Laser object with scan parameters
        span: Scan spanning factor
        """
        self.laser = laser
        self.span = span
        self.scan_size = laser.scan_size
        self.distances_mm = np.zeros(laser.scan_size, dtype=np.int32)
        self.angles_deg = None
        
    def update(self, scans_mm, hole_width_mm, velocities=(0,0), scan_angles_degrees=None):
        """
        Update scan with new readings
        scans_mm: list or array of distance readings in mm
        hole_width_mm: width of holes/obstacles in mm
        velocities: tuple of (dxy_mm/dt, dtheta_degrees/dt)
        scan_angles_degrees: optional custom angles for the scan points
        """
        # Store distances
        self.distances_mm[:] = np.array(scans_mm, dtype=np.int32)
        
        # Store angles if provided, otherwise calculate default angles
        if scan_angles_degrees is not None:
            self.angles_deg = np.array(scan_angles_degrees, dtype=np.float32)
        else:
            detection_angle_rad = np.radians(self.laser.detection_angle_degrees)
            self.angles_deg = np.linspace(
                -detection_angle_rad/2, 
                detection_angle_rad/2, 
                self.scan_size
            ) * 180.0 / np.pi

class CoreSLAM(ABC):
    """Abstract base class for SLAM algorithms"""
    
    def __init__(self, laser, map_size_pixels, map_size_meters, map_quality=50, hole_width_mm=600):
        """
        Initialize the SLAM algorithm
        laser: A Laser object with sensor parameters
        map_size_pixels: Size of the map in pixels
        map_size_meters: Size of the map in meters
        map_quality: Quality of map updates (0-255)
        hole_width_mm: Width of obstacles in mm
        """
        from .maps import OccupancyGridMap
        
        self.laser = laser
        self.map_quality = map_quality
        self.hole_width_mm = hole_width_mm
        
        # Create the map
        self.map = OccupancyGridMap(map_size_pixels, map_size_meters)
        
        # Create scans for distance calculation and map building
        self.scan_for_distance = Scan(laser, 1)
        self.scan_for_mapbuild = Scan(laser, 3)
    
    @abstractmethod
    def update(self, scans_mm, pose_change, scan_angles_degrees=None, should_update_map=True):
        """
        Update the SLAM algorithm with new scan and odometry data
        scans_mm: LiDAR distance readings in mm
        pose_change: tuple (dxy_mm, dtheta_degrees, dt_seconds)
        scan_angles_degrees: optional angles for non-uniform scan points
        should_update_map: whether to update the map
        """
        pass
    
    def get_map(self):
        """Get the current map as a numpy array"""
        return self.map.get_map()

    def __str__(self):
        return f"SLAM: map size = {self.map.size_pixels}x{self.map.size_pixels} pixels, " \
               f"{self.map.size_meters}x{self.map.size_meters} meters, " \
               f"quality = {self.map.quality}/255, hole width = {self.hole_width_mm} mm"