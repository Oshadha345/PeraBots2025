class Laser:
    """Base class for laser rangefinder (LiDAR) sensors"""
    
    def __init__(self, scan_size, scan_rate_hz, detection_angle_degrees, 
                 distance_no_detection_mm, detection_margin=0, offset_mm=0):
        """
        Initialize a laser sensor model
        scan_size: Number of points per scan
        scan_rate_hz: Scan rate in Hz
        detection_angle_degrees: Angular range of the scan
        distance_no_detection_mm: Value returned when nothing detected
        detection_margin: Detection margin for edge filtering
        offset_mm: Offset of the sensor from robot center
        """
        self.scan_size = scan_size
        self.scan_rate_hz = scan_rate_hz
        self.detection_angle_degrees = detection_angle_degrees
        self.distance_no_detection_mm = distance_no_detection_mm
        self.detection_margin = detection_margin
        self.offset_mm = offset_mm
    
    def __str__(self):
        return (f"Laser: scan_size={self.scan_size} | "
                f"scan_rate={self.scan_rate_hz:.3f} Hz | "
                f"detection_angle={self.detection_angle_degrees:.3f} deg | "
                f"distance_no_detection={self.distance_no_detection_mm:.4f} mm | "
                f"detection_margin={self.detection_margin} | "
                f"offset={self.offset_mm:.4f} mm")

# Common LiDAR models
class RPLidarA1(Laser):
    """SLAMTEC RPLidar A1 model"""
    
    def __init__(self, detection_margin=0, offset_mm=0):
        super().__init__(360, 5.5, 360, 12000, detection_margin, offset_mm)

class URG04LX(Laser):
    """Hokuyo URG-04LX model"""
    
    def __init__(self, detection_margin=0, offset_mm=0):
        super().__init__(682, 10, 240, 4000, detection_margin, offset_mm)

class XVLidar(Laser):
    """GetSurreal XVLidar model"""
    
    def __init__(self, detection_margin=0, offset_mm=0):
        super().__init__(360, 5.5, 360, 6000, detection_margin, offset_mm)

class CustomLidar(Laser):
    """Custom lidar for user-defined parameters"""
    
    def __init__(self, scan_size, scan_rate_hz, detection_angle_degrees, 
                 distance_no_detection_mm, detection_margin=0, offset_mm=0):
        super().__init__(scan_size, scan_rate_hz, detection_angle_degrees,
                        distance_no_detection_mm, detection_margin, offset_mm)