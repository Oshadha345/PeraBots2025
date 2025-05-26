"""
Configuration parameters for the Webots simulation
"""

class SimulationConfig:
    def __init__(self):
        # Robot parameters
        self.WHEEL_RADIUS = 0.05  # meters
        self.WHEEL_DISTANCE = 0.18  # meters
        self.MAX_MOTOR_SPEED = 10.0  # rad/s
        
        # PID controller parameters
        self.PID_KP = 0.5
        self.PID_KI = 0.0
        self.PID_KD = 0.05
        
        # Sensor noise parameters for realism
        self.ACCEL_NOISE_STD = 0.01  # m/s^2
        self.GYRO_NOISE_STD = 0.001  # rad/s
        self.ENCODER_NOISE_STD = 0.0001  # rad
        
        # SLAM parameters
        self.MAP_SIZE_PIXELS = 500
        self.MAP_SIZE_METERS = 10.0
        self.MAP_QUALITY = 50
        self.HOLE_WIDTH_MM = 600
        self.NUM_PARTICLES = 100
        self.LIDAR_OFFSET_MM = 0
        
        # Navigation parameters
        self.WAYPOINT_THRESHOLD = 0.1  # meters
        self.GOAL_THRESHOLD = 0.2  # meters
        self.OBSTACLE_THRESHOLD = 0.5  # meters
        self.PATH_RESOLUTION = 0.1  # meters
        
        # Visualization parameters
        self.DISPLAY_WIDTH = 500
        self.DISPLAY_HEIGHT = 500
        self.PIXELS_PER_METER = 50  # pixels per meter for visualization
        
        # Simulation parameters
        self.SIM_SPEEDUP = 1.0  # Simulation speed multiplier