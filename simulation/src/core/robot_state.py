# simulation/src/core/robot_state.py
import numpy as np

class RobotState:
    """Represents the robot's state in the simulation"""
    
    def __init__(self):
        """Initialize robot state with default values"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.lidar_scan = []
        self.timestamp = 0.0
        
    def update(self, x=None, y=None, theta=None, velocity=None, angular_velocity=None, lidar_scan=None, timestamp=None):
        """Update robot state with new values"""
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if theta is not None:
            self.theta = theta
        if velocity is not None:
            self.velocity = velocity
        if angular_velocity is not None:
            self.angular_velocity = angular_velocity
        if lidar_scan is not None:
            self.lidar_scan = lidar_scan
        if timestamp is not None:
            self.timestamp = timestamp

    def as_dict(self):
        return {
            'timestamp': self.timestamp,
            'x': self.x,
            'y': self.y,
            'theta': self.theta,
            'velocity': self.velocity,
            'angular_velocity': self.angular_velocity,
            'lidar_scan': self.lidar_scan,
            'imu': self.imu,
            'encoder': self.encoder
        }