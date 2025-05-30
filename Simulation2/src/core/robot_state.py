class RobotState:
    def __init__(self):
        self.x = 0.0  # X position of the robot
        self.y = 0.0  # Y position of the robot
        self.theta = 0.0  # Orientation of the robot in radians
        self.velocity = 0.0  # Linear velocity of the robot
        self.angular_velocity = 0.0  # Angular velocity of the robot
        self.lidar_scan = []  # LiDAR scan data

    def update(self, x, y, theta, velocity, angular_velocity, lidar_scan):
        """Update the robot's state with new sensor readings."""
        self.x = x
        self.y = y
        self.theta = theta
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.lidar_scan = lidar_scan

    def get_position(self):
        """Return the current position of the robot."""
        return self.x, self.y

    def get_orientation(self):
        """Return the current orientation of the robot."""
        return self.theta

    def get_velocity(self):
        """Return the current velocity of the robot."""
        return self.velocity

    def get_lidar_data(self):
        """Return the current LiDAR scan data."""
        return self.lidar_scan