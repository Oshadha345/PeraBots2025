class RobotConfig:
    """Configuration parameters for the robot"""
    
    # Robot physical parameters
    WHEEL_RADIUS = 0.05  # meters
    WHEEL_DISTANCE = 0.18  # meters
    MAX_MOTOR_SPEED = 10.0  # rad/s
    ROBOT_LENGTH = 0.3  # meters
    ROBOT_WIDTH = 0.2  # meters
    
    # Sensor parameters
    LIDAR_MAX_RANGE = 5.0  # meters
    LIDAR_MIN_RANGE = 0.05  # meters
    LIDAR_ANGULAR_RESOLUTION = 1.0  # degrees
    LIDAR_OFFSET_X = 0.0  # meters (forward offset from center)
    LIDAR_OFFSET_Y = 0.0  # meters (lateral offset from center)
    
    # Control parameters
    PID_KP = 0.5  # Proportional gain
    PID_KI = 0.0  # Integral gain
    PID_KD = 0.05  # Derivative gain
    
    # Navigation parameters
    WAYPOINT_THRESHOLD = 0.1  # meters
    GOAL_THRESHOLD = 0.2  # meters
    OBSTACLE_THRESHOLD = 0.5  # meters
    
    # SLAM parameters
    MAP_SIZE_PIXELS = 500
    MAP_SIZE_METERS = 10.0
    MAP_QUALITY = 50
    HOLE_WIDTH_MM = 600
    NUM_PARTICLES = 100
    
    # Visualization parameters
    DISPLAY_WIDTH = 500
    DISPLAY_HEIGHT = 500
    
    # Simulation parameters
    SIM_SPEEDUP = 1.0  # Simulation speed multiplier