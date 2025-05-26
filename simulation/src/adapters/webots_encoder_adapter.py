"""
Adapter for Webots encoder (position sensor)
Bridges between Webots position sensors and our software's encoder interface
"""

import math

class WebotsEncoderAdapter:
    def __init__(self, left_encoder, right_encoder, wheel_radius, wheel_distance):
        """
        Initialize the encoder adapter
        
        Args:
            left_encoder: Left wheel position sensor
            right_encoder: Right wheel position sensor
            wheel_radius: Radius of the wheels in meters
            wheel_distance: Distance between wheels in meters
        """
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        
        # Previous values for calculating deltas
        self.prev_left_pos = 0.0
        self.prev_right_pos = 0.0
        
        # Odometry values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        print("[INFO] Encoder adapter initialized")

    def get_data(self):
        """
        Get the current encoder data
        
        Returns:
            Dictionary with encoder data and derived odometry
        """
        # Get current wheel positions
        left_pos = self.left_encoder.getValue()
        right_pos = self.right_encoder.getValue()
        
        # Calculate change in position
        delta_left = left_pos - self.prev_left_pos
        delta_right = right_pos - self.prev_right_pos
        
        # Update previous positions
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        
        # Calculate linear displacement of each wheel
        left_distance = delta_left * self.wheel_radius
        right_distance = delta_right * self.wheel_radius
        
        # Calculate robot displacement and rotation
        distance = (left_distance + right_distance) / 2.0
        dtheta = (right_distance - left_distance) / self.wheel_distance
        
        # Calculate velocities
        left_velocity = delta_left * self.wheel_radius  # This is actually a delta, not a rate
        right_velocity = delta_right * self.wheel_radius
        velocity = (left_velocity + right_velocity) / 2.0
        
        # Update odometry
        dx = distance * math.cos(self.theta + dtheta/2.0)
        dy = distance * math.sin(self.theta + dtheta/2.0)
        
        self.x += dx
        self.y += dy
        self.theta += dtheta
        
        # Normalize theta to [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Package data
        encoder_data = {
            'left_position': left_pos,
            'right_position': right_pos,
            'left_delta': delta_left,
            'right_delta': delta_right,
            'left_distance': left_distance,
            'right_distance': right_distance,
            'distance': distance,
            'dtheta': dtheta,
            'left_velocity': left_velocity,
            'right_velocity': right_velocity,
            'velocity': velocity,
            'x': self.x,
            'y': self.y,
            'theta': self.theta
        }
        
        return encoder_data

    def get_ticks(self):
        """
        Get raw encoder ticks
        
        Returns:
            Tuple of (left_ticks, right_ticks)
        """
        # Convert continuous position to simulated ticks
        ticks_per_revolution = 360  # Arbitrary value
        left_ticks = int(self.left_encoder.getValue() * ticks_per_revolution / (2 * math.pi))
        right_ticks = int(self.right_encoder.getValue() * ticks_per_revolution / (2 * math.pi))
        
        return (left_ticks, right_ticks)

    def get_distance(self):
        """
        Get distance traveled
        
        Returns:
            Distance traveled in meters
        """
        # Calculate odometry
        left_pos = self.left_encoder.getValue() * self.wheel_radius
        right_pos = self.right_encoder.getValue() * self.wheel_radius
        
        # Return average distance traveled by both wheels
        return (left_pos + right_pos) / 2.0

    def reset(self):
        """Reset odometry"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Note: We can't reset the Webots encoder values,
        # but we can reset our tracked values
        self.prev_left_pos = self.left_encoder.getValue()
        self.prev_right_pos = self.right_encoder.getValue()