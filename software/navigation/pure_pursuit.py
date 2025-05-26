import math
import numpy as np
from typing import List, Tuple, Optional

class PurePursuit:
    """
    Pure Pursuit path tracking controller.
    
    This controller looks ahead on the path to select a target point,
    then calculates the steering command needed to follow the path.
    """
    
    def __init__(self, lookahead_distance: float = 0.3, max_angular_velocity: float = 2.0):
        """
        Initialize the Pure Pursuit controller.
        
        Args:
            lookahead_distance: Distance to look ahead on the path (meters)
            max_angular_velocity: Maximum allowed angular velocity (rad/s)
        """
        self.lookahead_distance = lookahead_distance
        self.max_angular_velocity = max_angular_velocity
        self.path = []
        self.current_target_idx = 0
    
    def set_path(self, path: List[Tuple[float, float]]):
        """Set the path to follow."""
        self.path = path
        self.current_target_idx = 0
    
    def update(self, robot_x: float, robot_y: float, robot_heading: float) -> Tuple[float, float, bool]:
        """
        Update the controller based on current robot position.
        
        Args:
            robot_x: Current robot x position
            robot_y: Current robot y position
            robot_heading: Current robot heading (radians)
            
        Returns:
            (linear_velocity, angular_velocity, path_completed)
        """
        if not self.path or self.current_target_idx >= len(self.path):
            return 0.0, 0.0, True
        
        # Find the lookahead point on the path
        target_x, target_y, self.current_target_idx, end_reached = self._find_lookahead_point(
            robot_x, robot_y, self.lookahead_distance, self.current_target_idx
        )
        
        if end_reached and self._distance(robot_x, robot_y, self.path[-1][0], self.path[-1][1]) < 0.1:
            # We've reached the end of the path
            return 0.0, 0.0, True
        
        # Transform target to robot-relative coordinates
        target_rel_x, target_rel_y = self._transform_to_robot_frame(
            target_x, target_y, robot_x, robot_y, robot_heading
        )
        
        # Calculate curvature (1/radius)
        curvature = self._calculate_curvature(target_rel_x, target_rel_y)
        
        # Calculate velocities
        linear_velocity = 0.2  # m/s, could be dynamic based on curvature
        angular_velocity = linear_velocity * curvature
        
        # Limit angular velocity
        angular_velocity = max(min(angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)
        
        return linear_velocity, angular_velocity, False
    
    def _find_lookahead_point(
        self, robot_x: float, robot_y: float, lookahead_distance: float, start_idx: int
    ) -> Tuple[float, float, int, bool]:
        """
        Find the point on the path that is lookahead_distance away from the robot.
        
        Returns:
            (target_x, target_y, new_idx, end_reached)
        """
        # If we're at the end of the path, return the last point
        if start_idx >= len(self.path) - 1:
            return self.path[-1][0], self.path[-1][1], len(self.path) - 1, True
        
        # Find the first point on the path that is at least lookahead_distance away
        for i in range(start_idx, len(self.path) - 1):
            start_x, start_y = self.path[i]
            end_x, end_y = self.path[i + 1]
            
            # Check if the lookahead circle intersects with this path segment
            intersections = self._circle_line_intersection(
                robot_x, robot_y, lookahead_distance,
                start_x, start_y, end_x, end_y
            )
            
            if intersections:
                # Return the farthest intersection point
                if len(intersections) == 1:
                    return intersections[0][0], intersections[0][1], i, False
                else:
                    # Return the second intersection (farther along the path)
                    return intersections[1][0], intersections[1][1], i, False
            
            # If we didn't find an intersection but the next point is far enough,
            # we can just use the next point
            if self._distance(robot_x, robot_y, end_x, end_y) >= lookahead_distance:
                return end_x, end_y, i + 1, False
        
        # If we reach here, we're near the end of the path
        return self.path[-1][0], self.path[-1][1], len(self.path) - 1, True
    
    def _circle_line_intersection(
        self, circle_x: float, circle_y: float, radius: float,
        line_start_x: float, line_start_y: float, 
        line_end_x: float, line_end_y: float
    ) -> List[Tuple[float, float]]:
        """Find the intersection points of a circle and a line segment."""
        # Vector from start to end
        dx = line_end_x - line_start_x
        dy = line_end_y - line_start_y
        
        # Vector from circle center to line start
        a = circle_x - line_start_x
        b = circle_y - line_start_y
        
        # Quadratic equation coefficients
        a_coef = dx*dx + dy*dy
        b_coef = 2 * (dx*a + dy*b)
        c_coef = a*a + b*b - radius*radius
        
        discriminant = b_coef*b_coef - 4*a_coef*c_coef
        
        if discriminant < 0:
            # No intersection
            return []
        
        # Calculate intersection points
        t1 = (-b_coef + math.sqrt(discriminant)) / (2 * a_coef)
        t2 = (-b_coef - math.sqrt(discriminant)) / (2 * a_coef)
        
        intersections = []
        
        # Check if t1 is within [0,1] (on the line segment)
        if 0 <= t1 <= 1:
            x1 = line_start_x + t1 * dx
            y1 = line_start_y + t1 * dy
            intersections.append((x1, y1))
            
        # Check if t2 is within [0,1] (on the line segment)
        if 0 <= t2 <= 1:
            x2 = line_start_x + t2 * dx
            y2 = line_start_y + t2 * dy
            intersections.append((x2, y2))
            
        return intersections
    
    def _transform_to_robot_frame(
        self, point_x: float, point_y: float, 
        robot_x: float, robot_y: float, robot_heading: float
    ) -> Tuple[float, float]:
        """Transform a point from world coordinates to robot-relative coordinates."""
        # Translate point to make robot the origin
        x_translated = point_x - robot_x
        y_translated = point_y - robot_y
        
        # Rotate by -heading to align with robot's orientation
        cos_heading = math.cos(-robot_heading)
        sin_heading = math.sin(-robot_heading)
        
        x_rotated = x_translated * cos_heading - y_translated * sin_heading
        y_rotated = x_translated * sin_heading + y_translated * cos_heading
        
        return x_rotated, y_rotated
    
    def _calculate_curvature(self, target_x: float, target_y: float) -> float:
        """
        Calculate the curvature needed to reach the target point.
        For a differential drive robot, this translates to angular velocity.
        """
        # Special case: target is directly ahead or behind
        if abs(target_x) < 1e-6:
            return 0.0
            
        # Pure pursuit curvature formula
        curvature = 2 * target_y / (target_x**2 + target_y**2)
        return curvature
    
    def _distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate Euclidean distance between two points."""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)