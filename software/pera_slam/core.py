import numpy as np
import math

class Position:
    """
    Represents a position and orientation in 2D space
    """
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        # Add aliases for compatibility
        self.x_mm = x
        self.y_mm = y
        self.theta_degrees = theta
    
    def distance_to(self, other_position):
        """Calculate Euclidean distance to another position"""
        return math.sqrt((self.x - other_position.x)**2 + 
                         (self.y - other_position.y)**2)
    
    def copy(self):
        return Position(self.x_mm, self.y_mm, self.theta_degrees)

class CoreSLAM:
    """
    Base class for SLAM algorithms
    """
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                 map_quality=50, hole_width_mm=600):
        # Store parameters
        self.laser = laser
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.map_quality = map_quality
        self.hole_width_mm = hole_width_mm
        
        # Initialize the occupancy grid map
        self.map = np.zeros((map_size_pixels, map_size_pixels))
        
        # Calculate pixels per meter for conversion
        self.pixels_per_meter = map_size_pixels / map_size_meters
        
        # Initialize robot position
        self.position = Position(0, 0, 0)
        
    def update(self, dxy_mm, dtheta_degrees, scans_mm, scan_angles_degrees):
        """
        Update the map and position based on new scan data
        """
        # Base implementation - should be overridden
        pass
    
    def get_map(self):
        """Return the current occupancy grid map"""
        return self.map
    
    def get_position(self):
        """Return the current estimated position as (x_mm, y_mm, theta_degrees)"""
        return (self.position.x, self.position.y, self.position.theta)