import numpy as np

class OccupancyGridMap:
    """2D occupancy grid map for SLAM"""
    
    def __init__(self, size_pixels, size_meters):
        """
        Initialize an empty map
        size_pixels: width/height of map in pixels
        size_meters: width/height of map in meters
        """
        self.size_pixels = size_pixels
        self.size_meters = size_meters
        self.resolution = size_meters / size_pixels  # meters per pixel
        
        # Initialize empty grid (0 = unknown, >0 = occupied, <0 = free)
        # Using log-odds representation for occupancy
        self.grid = np.zeros((size_pixels, size_pixels), dtype=np.float32)
        
        # Pre-compute coordinates for distance calculations
        self.grid_coords = np.mgrid[0:size_pixels, 0:size_pixels].astype(np.float32)
    
    def get_map(self):
        """Get probability map from log-odds grid"""
        return 1.0 - 1.0 / (1.0 + np.exp(self.grid))
    
    def set_map(self, probabilities):
        """Set map from probability values (0-1)"""
        valid_probs = np.clip(probabilities, 0.001, 0.999)
        self.grid = np.log(valid_probs / (1.0 - valid_probs))
    
    def update(self, scan, position, quality, hole_width_mm):
        """
        Update map based on lidar scan and robot position
        scan: Scan object with distance and angle data
        position: Position object with robot pose
        quality: Update quality (0-255)
        hole_width_mm: Width of obstacles in mm
        """
        # Convert position to map coordinates
        position_pix_x = position.x_mm / (self.size_meters * 1000) * self.size_pixels
        position_pix_y = position.y_mm / (self.size_meters * 1000) * self.size_pixels
        
        # Convert quality to log-odds update weight
        quality_factor = quality / 255.0
        log_odds_update = np.log(quality_factor / (1.0 - quality_factor))
        
        # Ray tracing algorithm to update map
        for i in range(len(scan.distances_mm)):
            if scan.distances_mm[i] <= 0:
                continue  # Skip invalid measurements
                
            # Calculate endpoint in world coordinates
            angle_rad = np.radians(position.theta_degrees + scan.angles_deg[i])
            dist_m = scan.distances_mm[i] / 1000.0  # convert to meters
            
            endpoint_x = position.x_mm/1000.0 + np.cos(angle_rad) * dist_m
            endpoint_y = position.y_mm/1000.0 + np.sin(angle_rad) * dist_m
            
            # Convert to map coordinates
            endpoint_pix_x = endpoint_x / self.size_meters * self.size_pixels
            endpoint_pix_y = endpoint_y / self.size_meters * self.size_pixels
            
            # Bresenham's line algorithm
            points = self._bresenham(
                int(position_pix_x), 
                int(position_pix_y),
                int(endpoint_pix_x), 
                int(endpoint_pix_y)
            )
            
            # Mark path as free space
            for j in range(len(points) - 1):
                x, y = points[j]
                if 0 <= x < self.size_pixels and 0 <= y < self.size_pixels:
                    self.grid[x, y] -= log_odds_update * 0.5  # free space
            
            # Mark endpoint as occupied
            if 0 <= points[-1][0] < self.size_pixels and 0 <= points[-1][1] < self.size_pixels:
                self.grid[points[-1][0], points[-1][1]] += log_odds_update
        
        # Clip values to prevent numerical issues
        np.clip(self.grid, -100, 100, out=self.grid)
    
    def _bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for grid traversal"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return points
    
    def world_to_map(self, x_mm, y_mm):
        """Convert world coordinates (mm) to map coordinates (pixels)"""
        x_pix = int(x_mm / (self.size_meters * 1000) * self.size_pixels)
        y_pix = int(y_mm / (self.size_meters * 1000) * self.size_pixels)
        return x_pix, y_pix
    
    def map_to_world(self, x_pix, y_pix):
        """Convert map coordinates (pixels) to world coordinates (mm)"""
        x_mm = x_pix * (self.size_meters * 1000) / self.size_pixels
        y_mm = y_pix * (self.size_meters * 1000) / self.size_pixels
        return x_mm, y_mm