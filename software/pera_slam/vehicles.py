import math
from abc import ABC, abstractmethod

class Vehicle(ABC):
    """Abstract base class for all vehicle types"""
    
    @abstractmethod
    def compute_pose_change(self, *args, **kwargs):
        """Compute pose change from sensor readings"""
        pass

class WheeledVehicle(Vehicle):
    """Class for differential-drive wheeled robots"""
    
    def __init__(self, wheel_radius_mm, half_axle_length_mm):
        """
        Initialize a wheeled vehicle model
        wheel_radius_mm: Radius of the wheels in mm
        half_axle_length_mm: Half the distance between the wheels in mm
        """
        self.wheel_radius_mm = wheel_radius_mm
        self.half_axle_length_mm = half_axle_length_mm
        
        self.timestamp_seconds_prev = None
        self.left_wheel_degrees_prev = None
        self.right_wheel_degrees_prev = None
    
    def compute_pose_change(self, timestamp, left_wheel, right_wheel):
        """
        Compute pose change based on wheel odometry
        timestamp: Current timestamp
        left_wheel: Left wheel reading
        right_wheel: Right wheel reading
        
        Returns: (dxy_mm, dtheta_degrees, dt_seconds)
        """
        dxy_mm = 0
        dtheta_degrees = 0
        dt_seconds = 0
        
        # Extract odometry readings (must be implemented by subclasses)
        timestamp_seconds, left_wheel_degrees, right_wheel_degrees = \
            self.extract_odometry(timestamp, left_wheel, right_wheel)
        
        if self.timestamp_seconds_prev is not None:
            # Calculate wheel movements
            left_diff_degrees = left_wheel_degrees - self.left_wheel_degrees_prev
            right_diff_degrees = right_wheel_degrees - self.right_wheel_degrees_prev
            
            # Calculate linear movement
            dxy_mm = self.wheel_radius_mm * \
                    (math.radians(left_diff_degrees) + math.radians(right_diff_degrees))
            
            # Calculate angular movement
            dtheta_degrees = (float(self.wheel_radius_mm) / self.half_axle_length_mm) * \
                    (right_diff_degrees - left_diff_degrees)
            
            # Calculate time difference
            dt_seconds = timestamp_seconds - self.timestamp_seconds_prev
        
        # Store current readings for next time
        self.timestamp_seconds_prev = timestamp_seconds
        self.left_wheel_degrees_prev = left_wheel_degrees
        self.right_wheel_degrees_prev = right_wheel_degrees
        
        return dxy_mm, dtheta_degrees, dt_seconds
    
    @abstractmethod
    def extract_odometry(self, timestamp, left_wheel, right_wheel):
        """
        Extract standardized odometry from robot-specific readings
        Must be implemented by subclasses
        
        Returns: (timestamp_seconds, left_wheel_degrees, right_wheel_degrees)
        """
        pass
    
    def __str__(self):
        return f"WheeledVehicle: wheel_radius={self.wheel_radius_mm:.2f} mm, " \
               f"half_axle_length={self.half_axle_length_mm:.2f} mm"