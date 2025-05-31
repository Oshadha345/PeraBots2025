# simulation/src/pera_slam/slam.py
import sys
import os 

# Ensure the parent directory is in the path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parent_dir)

import numpy as np
from .maps import OccupancyGridMap
from .position import Position

class DummyScan:
    """Wrapper for LiDAR scan data"""
    
    def __init__(self, size):
        self.size = size
        self.distances_mm = np.ones(size) * 1000  # Default 1m
        self.angles_deg = np.linspace(-180, 180, size)
        
    def update(self, scans_mm, hole_width_mm=600):
        """Update scan data"""
        if scans_mm is not None and len(scans_mm) > 0:
            self.distances_mm = np.array(scans_mm)
            # Regenerate angles if needed
            if len(self.angles_deg) != len(self.distances_mm):
                self.angles_deg = np.linspace(-180, 180, len(self.distances_mm))
        return self



class ParticleFilterSLAM:
    """Compatible SLAM implementation to work with existing controller"""
    
    def __init__(self, laser=None, map_size_pixels=500, map_size_meters=10.0, 
                 map_quality=50, hole_width_mm=600, num_particles=1):
        """Initialize SLAM system with controller-compatible parameters"""
        # Store configuration parameters
        self.config = {
            'map_size_pixels': map_size_pixels,
            'map_size_meters': map_size_meters,
            'map_quality': map_quality,
            'hole_width_mm': hole_width_mm,
            'num_particles': num_particles
        }
        
        # Store laser reference
        self.laser = laser
        
        # Create occupancy grid map
        self.map = OccupancyGridMap(
            size_pixels=map_size_pixels,
            size_meters=map_size_meters
        )
        
        # Create position tracker
        self.position = Position()
        
        # Simple tracking
        self.prev_pos_x = 0
        self.prev_pos_y = 0
        self.prev_theta = 0
        self.first_update = True
        
        # Add necessary attributes that controller expects
        self.scan_for_distance = None
        self.scan_for_mapbuild = None
        
        print(f"[INFO] SLAM initialized with map size: {map_size_pixels}x{map_size_pixels} pixels")
    
    def update(self, scans_mm, pose_change, scan_angles_degrees=None):
        """Update SLAM with new scan data and odometry
        
        Args:
            scans_mm: LiDAR scan data (distances in mm)
            pose_change: Tuple of (dxy_mm, dtheta_degrees, dt_seconds)
            scan_angles_degrees: Optional angles for scan points
        """
        try:
            # Extract pose change components
            dxy_mm, dtheta_degrees, dt_seconds = pose_change
            
            # Create scan data structure
            class ScanData:
                def __init__(self):
                    self.distances_mm = np.array(scans_mm) if isinstance(scans_mm, list) else scans_mm
                    if scan_angles_degrees is not None:
                        self.angles_deg = scan_angles_degrees
                    else:
                        # Generate evenly spaced angles
                        self.angles_deg = np.linspace(-180, 180, len(self.distances_mm))
            
            # Create scan object
            scan_data = ScanData()
            
            # Update laser with scan data if available
            if self.laser:
                self.laser.update(scan_data)
            
            # Update position based on pose change
            # Convert from incremental to absolute position
            dx_mm = dxy_mm * np.cos(np.radians(self.position.theta_degrees))
            dy_mm = dxy_mm * np.sin(np.radians(self.position.theta_degrees))
            
            # Update position
            self.position.update(
                x_mm=self.position.x_mm + dx_mm,
                y_mm=self.position.y_mm + dy_mm,
                theta_degrees=self.position.theta_degrees + dtheta_degrees
            )
            
            # Update map with scan data
            success = self.map.update(scan_data, self.position)
            
            # Store these for compatibility
            self.scan_for_distance = scan_data
            self.scan_for_mapbuild = scan_data
            
            return success
            
        except Exception as e:
            print(f"[ERROR] SLAM update failed: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def get_position(self):
        """Get current position estimate as tuple"""
        return (self.position.x_mm, self.position.y_mm, self.position.theta_degrees)
    
    def get_map(self):
        """Get current map"""
        if hasattr(self.map, 'get_map'):
            return self.map.get_map()
        return self.map
