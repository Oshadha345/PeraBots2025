class SimulationConfig:
    def __init__(self):
        # Sensor settings
        self.LIDAR_OFFSET_MM = 0.0
        self.WHEEL_RADIUS = 0.1  # Wheel radius in meters
        self.WHEEL_DISTANCE = 0.5  # Distance between wheels in meters
        
        # PID controller gains
        self.PID_KP = 1.0
        self.PID_KI = 0.0
        self.PID_KD = 0.0
        
        # Simulation parameters
        self.DISPLAY_WIDTH = 800
        self.DISPLAY_HEIGHT = 600
        self.MAX_MOTOR_SPEED = 6.28  # Maximum speed in m/s
        
        # Wall following parameters
        self.WALL_FOLLOWING_DISTANCE = 0.2  # Desired distance from the wall in meters
        self.AVOIDANCE_DISTANCE = 0.5  # Distance to trigger obstacle avoidance in meters
        self.WAYPOINT_THRESHOLD = 0.1  # Threshold to consider waypoint reached
        
        # SLAM parameters
        self.MAP_SIZE_PIXELS = 500
        self.MAP_SIZE_METERS = 10.0
        self.MAP_QUALITY = 0.5
        self.HOLE_WIDTH_MM = 50
        self.NUM_PARTICLES = 100  # Number of particles for SLAM