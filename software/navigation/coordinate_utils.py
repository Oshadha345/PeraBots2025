import math
import numpy as np
from typing import List, Tuple

def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate Euclidean distance between two points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def angle_diff(a1: float, a2: float) -> float:
    """Calculate the smallest angle difference between two angles in radians."""
    diff = (a2 - a1) % (2 * math.pi)
    if diff > math.pi:
        diff -= 2 * math.pi
    return diff

def world_to_robot_transform(
    world_x: float, world_y: float, 
    robot_x: float, robot_y: float, robot_heading: float
) -> Tuple[float, float]:
    """
    Transform a point from world coordinates to robot-relative coordinates.
    
    Args:
        world_x, world_y: The point in world coordinates
        robot_x, robot_y: The robot position in world coordinates
        robot_heading: The robot heading in world coordinates (radians)
        
    Returns:
        (x, y) coordinates relative to robot
    """
    # Translate
    dx = world_x - robot_x
    dy = world_y - robot_y
    
    # Rotate
    cos_h = math.cos(-robot_heading)
    sin_h = math.sin(-robot_heading)
    
    x_robot = dx * cos_h - dy * sin_h
    y_robot = dx * sin_h + dy * cos_h
    
    return (x_robot, y_robot)

def robot_to_world_transform(
    robot_x: float, robot_y: float,
    world_robot_x: float, world_robot_y: float, world_robot_heading: float
) -> Tuple[float, float]:
    """
    Transform a point from robot-relative coordinates to world coordinates.
    
    Args:
        robot_x, robot_y: The point in robot-relative coordinates
        world_robot_x, world_robot_y: The robot position in world coordinates
        world_robot_heading: The robot heading in world coordinates (radians)
        
    Returns:
        (x, y) coordinates in world frame
    """
    # Rotate
    cos_h = math.cos(world_robot_heading)
    sin_h = math.sin(world_robot_heading)
    
    x_rotated = robot_x * cos_h - robot_y * sin_h
    y_rotated = robot_x * sin_h + robot_y * cos_h
    
    # Translate
    x_world = x_rotated + world_robot_x
    y_world = y_rotated + world_robot_y
    
    return (x_world, y_world)