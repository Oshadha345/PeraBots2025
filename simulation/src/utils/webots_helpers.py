"""
Helper functions for the Webots simulation
"""

import numpy as np
import math
import os

def setup_display(robot, width, height):
    """
    Set up a display for visualization
    
    Args:
        robot: Webots Robot instance
        width: Display width
        height: Display height
        
    Returns:
        Display device or None if not available
    """
    try:
        display = robot.getDevice('display')
        if display:
            return display
    except:
        print("[WARNING] Display device not found")
    return None

def draw_robot(display, x, y, theta, size, color):
    """
    Draw the robot on the display
    
    Args:
        display: Webots Display device
        x, y: Robot position in pixels
        theta: Robot orientation in radians
        size: Robot size in pixels
        color: Robot color as an int
    """
    if display is None:
        return
        
    # Clear previous drawing
    display.setColor(0xFFFFFF)  # White background
    display.fillRectangle(0, 0, display.getWidth(), display.getHeight())
    
    # Draw robot body
    display.setColor(color)
    display.fillOval(int(x - size/2), int(y - size/2), size, size)
    
    # Draw direction indicator
    end_x = x + math.cos(theta) * size
    end_y = y + math.sin(theta) * size
    display.setColor(0xFF0000)  # Red for direction
    display.drawLine(int(x), int(y), int(end_x), int(end_y))

def draw_lidar_scan(display, x, y, scan_mm, angles_deg, scale, max_range_mm=5000):
    """
    Draw LiDAR scan on the display
    
    Args:
        display: Webots Display device
        x, y: Robot position in pixels
        scan_mm: LiDAR scan data in mm
        angles_deg: Scan angles in degrees
        scale: Scale factor (pixels per meter)
        max_range_mm: Maximum range to display
    """
    if display is None or not scan_mm:
        return
        
    display.setColor(0x00FF00)  # Green for LiDAR points
    
    for i, (distance, angle) in enumerate(zip(scan_mm, angles_deg)):
        if i % 5 == 0:  # Only draw every 5th point for clarity
            # Limit to max range
            distance = min(distance, max_range_mm)
            
            # Convert to meters and then to pixels
            distance_m = distance / 1000.0
            distance_px = distance_m * scale
            
            # Calculate endpoint
            angle_rad = math.radians(angle)
            end_x = x + math.cos(angle_rad) * distance_px
            end_y = y + math.sin(angle_rad) * distance_px
            
            # Draw point
            display.drawPixel(int(end_x), int(end_y))

def draw_path(display, path, robot_x, robot_y, scale, color=0x0000FF):
    """
    Draw a path on the display
    
    Args:
        display: Webots Display device
        path: List of (x, y) waypoints in meters
        robot_x, robot_y: Robot position in meters
        scale: Scale factor (pixels per meter)
        color: Path color as an int
    """
    if display is None or not path:
        return
        
    display.setColor(color)
    
    # Convert robot position to pixels
    robot_x_px = robot_x * scale + display.getWidth() / 2
    robot_y_px = robot_y * scale + display.getHeight() / 2
    
    # Draw path
    for i in range(len(path) - 1):
        # Convert waypoints to pixels
        x1 = path[i][0] * scale + display.getWidth() / 2
        y1 = path[i][1] * scale + display.getHeight() / 2
        x2 = path[i+1][0] * scale + display.getWidth() / 2
        y2 = path[i+1][1] * scale + display.getHeight() / 2
        
        # Draw line segment
        display.drawLine(int(x1), int(y1), int(x2), int(y2))
    
    # Draw waypoints
    for waypoint in path:
        x = waypoint[0] * scale + display.getWidth() / 2
        y = waypoint[1] * scale + display.getHeight() / 2
        display.fillOval(int(x) - 3, int(y) - 3, 6, 6)

def log_data(robot_state, slam=None, filename=None):
    """
    Log robot state and SLAM data to file
    
    Args:
        robot_state: RobotState instance
        slam: SLAM instance
        filename: Log filename
    """
    if filename is None:
        filename = "robot_log.csv"
        
    # Create logs directory if it doesn't exist
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
        
    filepath = os.path.join(log_dir, filename)
    
    # Check if file exists to write header
    file_exists = os.path.isfile(filepath)
    
    with open(filepath, 'a') as f:
        # Write header if file doesn't exist
        if not file_exists:
            header = "timestamp,x,y,theta,velocity,angular_velocity"
            if slam:
                header += ",slam_x,slam_y,slam_theta"
            f.write(header + "\n")
        
        # Write data
        data = f"{robot_state.timestamp},{robot_state.x},{robot_state.y},{robot_state.theta},"
        data += f"{robot_state.velocity},{robot_state.angular_velocity}"
        
        if slam:
            # Get SLAM position estimate
            x_mm, y_mm, theta_degrees = slam.get_position()
            data += f",{x_mm/1000.0},{y_mm/1000.0},{math.radians(theta_degrees)}"
            
        f.write(data + "\n")