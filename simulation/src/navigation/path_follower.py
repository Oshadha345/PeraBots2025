# simulation/src/navigation/path_follower.py
import numpy as np

class PathFollower:
    """Path following controller for robot navigation"""
    
    def __init__(self, motor_controller, lookahead=0.5, k_p=1.0, k_d=0.1):
        """Initialize path follower"""
        self.motor_controller = motor_controller
        self.lookahead = lookahead  # Look-ahead distance in meters
        self.k_p = k_p  # Proportional gain
        self.k_d = k_d  # Derivative gain
        self.prev_error = 0.0
        
    def compute_velocity(self, current_x, current_y, current_theta, target_x, target_y):
        """Compute linear and angular velocity to reach target"""
        # Calculate vector to target
        dx = target_x - current_x
        dy = target_y - current_y
        
        # Distance to target
        distance = np.sqrt(dx*dx + dy*dy)
        
        # Calculate target heading
        target_theta = np.arctan2(dy, dx)
        
        # Calculate heading error
        error = self._normalize_angle(target_theta - current_theta)
        
        # Calculate derivative of error
        d_error = error - self.prev_error
        self.prev_error = error
        
        # Calculate angular velocity using PD controller
        angular_velocity = self.k_p * error + self.k_d * d_error
        
        # Calculate linear velocity (slow down as we turn more or get closer)
        linear_velocity = 0.5 * (1.0 - abs(error) / np.pi) * min(distance, 1.0)
        
        return linear_velocity, angular_velocity
        
    def _normalize_angle(self, angle):
        """Normalize angle to range [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle