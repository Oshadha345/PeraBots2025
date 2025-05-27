"""
Utilities for visualizing robot state and environment
"""

import numpy as np
import math

class MapVisualizer:
    def __init__(self, display, map_size_pixels, map_size_meters, center_offset=True):
        """
        Initialize the map visualizer
        
        Args:
            display: Webots Display device
            map_size_pixels: Size of the map in pixels
            map_size_meters: Size of the map in meters
            center_offset: Whether to center the map on the display
        """
        self.display = display
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.scale = map_size_pixels / map_size_meters
        self.center_offset = center_offset
        
        if display:
            self.width = display.getWidth()
            self.height = display.getHeight()
            
            # Calculate offsets to center the map
            if center_offset:
                self.offset_x = (self.width - map_size_pixels) // 2
                self.offset_y = (self.height - map_size_pixels) // 2
            else:
                self.offset_x = 0
                self.offset_y = 0

    def draw_map(self, map_data, threshold=0.5):
        """
        Draw the occupancy grid map
        
        Args:
            map_data: 2D numpy array with occupancy probabilities (0-1)
            threshold: Threshold value for considering a cell occupied
        """
        if self.display is None or map_data is None:
            return
            
        # Create a copy to avoid modifying the original
        map_copy = map_data.copy()
        
        # Convert to grayscale values (0-255)
        map_image = np.zeros((self.map_size_pixels, self.map_size_pixels), dtype=np.uint8)
        
        # Free space (< threshold) is white, occupied space is black
        map_image[map_copy < threshold] = 255
        map_image[map_copy >= threshold] = 0
        
        # Draw the map pixel by pixel
        for y in range(self.map_size_pixels):
            for x in range(self.map_size_pixels):
                pixel_value = map_image[y, x]
                color = (pixel_value << 16) | (pixel_value << 8) | pixel_value
                self.display.setColor(color)
                self.display.drawPixel(x + self.offset_x, y + self.offset_y)

    def draw_robot(self, x_m, y_m, theta_rad, color=0xFF0000, size=10):
        """
        Draw the robot on the map
        
        Args:
            x_m, y_m: Robot position in meters
            theta_rad: Robot orientation in radians
            color: Robot color as an int
            size: Robot size in pixels
        """
        if self.display is None:
            return
            
        # Convert from meters to pixels
        x_px = int(x_m * self.scale) + self.offset_x
        y_px = int(y_m * self.scale) + self.offset_y
        
        # Draw robot as a circle
        self.display.setColor(color)
        self.display.fillOval(x_px - size//2, y_px - size//2, size, size)
        
        # Draw heading line
        end_x = x_px + int(math.cos(theta_rad) * size)
        end_y = y_px + int(math.sin(theta_rad) * size)
        self.display.drawLine(x_px, y_px, end_x, end_y)

    def draw_particles(self, particles, color=0x0000FF, size=2):
        """
        Draw particles from particle filter SLAM
        
        Args:
            particles: List of particles with x_mm, y_mm, theta properties
            color: Particle color as an int
            size: Particle size in pixels
        """
        if self.display is None or not particles:
            return
            
        self.display.setColor(color)
        
        for particle in particles:
            # Convert from mm to m, then to pixels
            x_px = int((particle.x_mm / 1000.0) * self.scale) + self.offset_x
            y_px = int((particle.y_mm / 1000.0) * self.scale) + self.offset_y
            
            # Draw particle as a small point
            self.display.fillOval(x_px - size//2, y_px - size//2, size, size)

    def draw_path(self, path, color=0x00FF00, line_width=2):
        """
        Draw a path on the map
        
        Args:
            path: List of (x, y) waypoints in meters
            color: Path color as an int
            line_width: Width of the path line
        """
        if self.display is None or not path:
            return
            
        self.display.setColor(color)
        
        # Draw path segments
        for i in range(len(path) - 1):
            # Convert from meters to pixels
            x1 = int(path[i][0] * self.scale) + self.offset_x
            y1 = int(path[i][1] * self.scale) + self.offset_y
            x2 = int(path[i+1][0] * self.scale) + self.offset_x
            y2 = int(path[i+1][1] * self.scale) + self.offset_y
            
            # Draw line segment
            self.display.drawLine(x1, y1, x2, y2)
            
        # Draw waypoints
        for waypoint in path:
            x = int(waypoint[0] * self.scale) + self.offset_x
            y = int(waypoint[1] * self.scale) + self.offset_y
            self.display.fillOval(x - 3, y - 3, 6, 6)

    def clear(self):
        """Clear the display"""
        if self.display:
            self.display.setColor(0xFFFFFF)  # White
            self.display.fillRectangle(0, 0, self.width, self.height)