import time

class RobotState:
    """
    Holds the robot's current estimated state for use in control, SLAM, and logging.
    """
    def __init__(self):
        self.timestamp = time.time()
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians
        self.velocity = 0.0  # m/s
        self.angular_velocity = 0.0  # rad/s
        self.lidar_scan = []  # list of distances (mm or m)
        self.imu = {
            'accel': [0.0, 0.0, 0.0],
            'gyro': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0]
        }
        self.encoder = {
            'left': 0.0,
            'right': 0.0
        }

    def update(self, x=None, y=None, theta=None, velocity=None, angular_velocity=None, lidar_scan=None, imu=None, encoder=None):
        self.timestamp = time.time()
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
        if imu is not None:
            self.imu = imu
        if encoder is not None:
            self.encoder = encoder

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