# simulation/src/pera_slam/lidar.py
import numpy as np

class RPLidarA1:
    """Adapter for RPLidar A1 sensor in simulation"""
    
    def __init__(self, offset_mm=0):
        """Initialize the LiDAR model"""
        self.offset_mm = offset_mm
        self.distances_mm = []
        self.angles_deg = []
        
        
    def process_scan(self, scan_data, angles=None):
        """Process raw scan data into distances and angles"""
        if angles is None:
            # Generate default angles if not provided
            angles = np.linspace(-180, 180, len(scan_data))
            
        # Apply offset if needed
        adjusted_scan = np.array(scan_data)
        
        return {
            'distances_mm': adjusted_scan,
            'angles_deg': angles
        }
    
    def update(self, scan_data):
        """Update with new scan data"""
        # Handle different scan data formats
        if hasattr(scan_data, 'distances_mm'):
            self.distances_mm = scan_data.distances_mm
            self.angles_deg = scan_data.angles_deg
        elif isinstance(scan_data, list) or isinstance(scan_data, np.ndarray):
            self.distances_mm = scan_data
            # Generate default angles if needed
            if len(self.angles_deg) != len(self.distances_mm):
                self.angles_deg = np.linspace(-180, 180, len(self.distances_mm))
        
        return True