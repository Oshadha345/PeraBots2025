import numpy as np

class OccupancyGridMap:
    """Simple occupancy grid map implementation"""
    
    def __init__(self, size_pixels=500, size_meters=10.0):
        # Basic properties
        self.size_pixels = size_pixels
        self.size_meters = size_meters
        self.resolution = size_meters / size_pixels
        
        # Map center
        self.origin_x = size_pixels // 2
        self.origin_y = size_pixels // 2
        
        # Initialize grid as numpy array (0.5 = unknown, 0 = free, 1 = occupied)
        self.grid = np.ones((size_pixels, size_pixels), dtype=np.float32) * 0.5
        
        # For comparison and shape access
        self.grid_array = self.grid
        self.shape = (size_pixels, size_pixels)
    
    def world_to_map(self, x, y):
        """Convert world coordinates (meters) to map coordinates (pixels)"""
        # Offset by origin and scale
        map_x = int(x / self.resolution + self.origin_x)
        map_y = int(y / self.resolution + self.origin_y)
        
        # Ensure within bounds
        map_x = max(0, min(self.size_pixels - 1, map_x))
        map_y = max(0, min(self.size_pixels - 1, map_y))
        
        return map_x, map_y
    
    def update(self, scan, position, quality=None, hole_width_mm=None):
        """Update map with scan data (simplified)"""
        try:
            # Print debug info
            print(f"Updating map: scan type={type(scan)}, position type={type(position)}")
            
            # Handle position
            if hasattr(position, 'x_mm') and hasattr(position, 'y_mm'):
                # Position object 
                pos_x = position.x_mm / 1000.0  # to meters
                pos_y = position.y_mm / 1000.0
                pos_theta = np.radians(position.theta_degrees)
            else:
                # Position tuple/list
                pos_x = position[0]
                pos_y = position[1] 
                pos_theta = position[2] if len(position) > 2 else 0.0
            
            # Handle scan - if it's a list/array convert to expected format
            if isinstance(scan, (list, np.ndarray)):
                # Create simple structure with distances and angles
                distances = np.array(scan, dtype=np.float32)
                angles = np.linspace(-180, 180, len(scan))
                
                # Simple class to hold scan data
                class SimpleScan:
                    def __init__(self):
                        self.distances_mm = distances
                        self.angles_deg = angles
                
                scan_data = SimpleScan()
            else:
                # Use provided scan object
                scan_data = scan
            
            # Mark robot position as free
            robot_x, robot_y = self.world_to_map(pos_x, pos_y)
            self.grid[robot_y, robot_x] = 0.0  # Free space
            
            # Process scan points
            for i in range(len(scan_data.distances_mm)):
                # Skip invalid readings
                if scan_data.distances_mm[i] <= 0:
                    continue
                
                # Convert to meters
                dist = scan_data.distances_mm[i] / 1000.0
                
                # Skip if too far
                if dist > self.size_meters / 2:
                    continue
                
                # Calculate endpoint
                angle = pos_theta + np.radians(scan_data.angles_deg[i])
                end_x = pos_x + dist * np.cos(angle)
                end_y = pos_y + dist * np.sin(angle)
                
                # Convert to map coordinates
                end_map_x, end_map_y = self.world_to_map(end_x, end_y)
                
                # Mark endpoint as occupied
                self.grid[end_map_y, end_map_x] = 1.0
                
                # Draw line to mark free space (simplified Bresenham)
                self._draw_line(robot_x, robot_y, end_map_x, end_map_y)
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to update map: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def _draw_line(self, x0, y0, x1, y1):
        """Mark free space along a line (simplified)"""
        # Maximum number of points to generate
        num_points = int(np.sqrt((x1-x0)**2 + (y1-y0)**2))
        
        # Avoid division by zero
        if num_points == 0:
            return
            
        # Generate points along the line
        for i in range(num_points):
            # Linear interpolation
            t = i / num_points
            x = int(x0 + t * (x1 - x0))
            y = int(y0 + t * (y1 - y0))
            
            # Check bounds
            if 0 <= x < self.size_pixels and 0 <= y < self.size_pixels:
                # Don't overwrite the endpoint
                if i < num_points - 1:
                    self.grid[y, x] = 0.0  # Mark as free
    
    def get_map(self):
        """Return current map"""
        return self.grid
        
    def get_binary_map(self, threshold=0.6):
        """Get binary version of map"""
        return (self.grid > threshold).astype(np.uint8)
        
    def __getattr__(self, name):
        """Handle any missing attributes by returning the grid"""
        print(f"[DEBUG] Accessing attribute {name} not found, returning grid")
        return getattr(self.grid, name)