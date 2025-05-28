"""
Two-Wheeled Robot Controller for Webots
This is the main controller script that integrates with our software modules
"""

from controller import Robot, Motor, DistanceSensor, InertialUnit, Lidar, PositionSensor
import sys
import os
import math
import numpy as np

# Add the src directory to the Python path for adapter imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))
# Add software directory to Python path
#sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'software'))
# Corrected path with the right number of parent directories
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'software'))
# Import adapters
from adapters.webots_lidar_adapter import WebotsLidarAdapter
from adapters.webots_imu_adapter import WebotsIMUAdapter


from adapters.webots_encoder_adapter import WebotsEncoderAdapter

# Import software modules
from core.robot_state import RobotState
from filters.complementary import ComplementaryFilter
from filters.ekf_filter import ExtendedKalmanFilter
from motor_control.pid_controller import PID
from pera_slam import ParticleFilterSLAM, RPLidarA1, visualize_map
from navigation.path_follower import PathFollower
from Path_planning.RRT import RRTStar

# Import configuration
from config.simulation_config import SimulationConfig

# Import utilities
from utils.webots_helpers import setup_display, log_data

class TwoWheelController:
    def __init__(self):
        """Initialize the controller"""
        # Create the Robot instance
        self.robot = Robot()
        
        # Load simulation configuration
        self.config = SimulationConfig()
        
        # Get the time step of the current world
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize sensors and motors
        self._init_devices()
        
        # Initialize adapters
        self._init_adapters()
        
        # Initialize filters
        self._init_filters()
        
        # Initialize SLAM
        self._init_slam()
        
        # Initialize path planning and following
        self._init_navigation()
        
        # Initialize robot state
        self.robot_state = RobotState()
        
        # Display for visualization
        self.display = setup_display(self.robot, self.config.DISPLAY_WIDTH, self.config.DISPLAY_HEIGHT)
        
        # Path tracking variables
        self.path = []
        self.current_path_index = 0
        self.target_position = None
        
        print("[INFO] Two Wheel Robot Controller initialized")

    def _init_devices(self):
        """Initialize robot devices (motors, sensors)"""
        # Initialize motors
        self.left_motor = self.robot.getDevice('left_wheel_motor')
        self.right_motor = self.robot.getDevice('right_wheel_motor')
        
        # Set motor parameters
        self.left_motor.setPosition(float('inf'))  # Set position control to velocity control
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Initialize position sensors (encoders)
        self.left_encoder = self.robot.getDevice('left_wheel_sensor')
        self.right_encoder = self.robot.getDevice('right_wheel_sensor')
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Initialize LiDAR
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        
        # Initialize IMU
        self.imu = self.robot.getDevice('inertial_unit')
        self.imu.enable(self.timestep)
        
        # Initialize accelerometer
        self.accelerometer = self.robot.getDevice('accelerometer')
        self.accelerometer.enable(self.timestep)
        
        # Initialize gyro
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timestep)

    def _init_adapters(self):
        """Initialize the sensor adapters"""
        # Create adapter instances
        self.lidar_adapter = WebotsLidarAdapter(self.lidar, self.config)
        self.imu_adapter = WebotsIMUAdapter(self.imu, self.accelerometer, self.gyro, self.config)
        self.encoder_adapter = WebotsEncoderAdapter(
            self.left_encoder, 
            self.right_encoder, 
            self.config.WHEEL_RADIUS, 
            self.config.WHEEL_DISTANCE
        )

    def _init_filters(self):
        """Initialize sensor fusion filters"""
        # Complementary filter for orientation
        self.comp_filter = ComplementaryFilter(alpha=0.98)
    
        # Extended Kalman Filter for pose estimation
        # For a 2D robot, state vector is [x, y, theta, v, omega]
        # Measurement vector is [encoder_left, encoder_right, orientation]
        self.ekf = ExtendedKalmanFilter(dim_x=5, dim_z=3)
    
        # Initialize EKF matrices
        self.ekf.F = np.eye(5)  # State transition matrix (identity to start)
        self.ekf.H = np.zeros((3, 5))  # Measurement matrix
    
        # Set up measurement matrix to extract relevant state variables
        self.ekf.H[0, 3] = 1.0  # First measurement (left encoder) relates to velocity
        self.ekf.H[1, 3] = 1.0  # Second measurement (right encoder) relates to velocity
        self.ekf.H[2, 2] = 1.0  # Third measurement (IMU) relates to theta
    
        # Process and measurement noise
        self.ekf.R = np.diag([0.1, 0.1, 0.05])  # Measurement noise
        self.ekf.Q = np.diag([0.01, 0.01, 0.01, 0.05, 0.05])  # Process noise
    
        # Initial state covariance
        self.ekf.P = np.eye(5) * 100  # High initial uncertainty
    
        # PID controllers for motor control
        self.left_pid = PID(self.config.PID_KP, self.config.PID_KI, self.config.PID_KD)
        self.right_pid = PID(self.config.PID_KP, self.config.PID_KI, self.config.PID_KD)

    def _init_slam(self):
        """Initialize SLAM module"""
        # Create a laser object for the SLAM algorithm
        self.laser = RPLidarA1(offset_mm=self.config.LIDAR_OFFSET_MM)
        
        # Create SLAM instance
        self.slam = ParticleFilterSLAM(
            laser=self.laser,
            map_size_pixels=self.config.MAP_SIZE_PIXELS,
            map_size_meters=self.config.MAP_SIZE_METERS,
            map_quality=self.config.MAP_QUALITY,
            hole_width_mm=self.config.HOLE_WIDTH_MM,
            num_particles=self.config.NUM_PARTICLES
        )

    def _init_navigation(self):
        """Initialize path planning and following modules"""
        # Path planning
        self.path_planner = None
        
        # Path following
        self.path_follower = PathFollower(motor_controller=self)

    def update_robot_state(self):
        """Update robot state with current sensor readings and estimated pose"""
        # Get sensor readings
        lidar_data = self.lidar_adapter.get_scan()
        imu_data = self.imu_adapter.get_data()
        encoder_data = self.encoder_adapter.get_data()
        
        # Update robot pose estimation using sensor fusion
        accel = imu_data['accel']
        gyro = imu_data['gyro']
        orientation = imu_data['orientation']
        
        # Use complementary filter for orientation
        filtered_orientation = self.comp_filter.update(accel, gyro, orientation)
        
        # Step 1: Update the state transition model based on time and current state
        dt = self.timestep / 1000.0  # Convert to seconds
        theta = float(self.ekf.x[2].item())  # Extract scalar value to avoid deprecation warning
        
        # Update state transition matrix for non-linear motion
        self.ekf.F[0, 3] = dt * np.cos(theta)  # x += v*dt*cos(theta)
        self.ekf.F[1, 3] = dt * np.sin(theta)  # y += v*dt*sin(theta)
        self.ekf.F[2, 4] = dt                  # theta += omega*dt
        
        # Define measurement function and its Jacobian
        def Hx(x):
            """Measurement function - converts state to measurement"""
            # Measurement vector: [velocity, angular_velocity, orientation]
            return np.array([
                x[3],                 # velocity
                x[4],                 # angular velocity
                x[2]                  # orientation (theta)
            ])
        
        def HJacobian(x):
            """Compute Jacobian of the measurement function"""
            # Jacobian is the derivative of Hx with respect to state variables
            # For our simple case, it's just a constant matrix
            H = np.zeros((3, 5))
            H[0, 3] = 1.0  # d(velocity measurement)/d(velocity state) = 1
            H[1, 4] = 1.0  # d(angular velocity measurement)/d(angular velocity state) = 1
            H[2, 2] = 1.0  # d(orientation measurement)/d(orientation state) = 1
            return H
        
        # Step 2: Perform prediction step (time update)
        self.ekf.predict()
        
        # Step 3: Create measurement vector from sensors
        z = np.array([
            encoder_data['left_velocity'],
            encoder_data['right_velocity'],
            filtered_orientation[2]  # Yaw/heading
        ])
        
        # Step 4: Perform correction step (measurement update) with required functions
        self.ekf.update(z, HJacobian, Hx)
        
        # Step 5: Extract the estimated state
        estimated_pose = [
            self.ekf.x[0],  # x position
            self.ekf.x[1],  # y position 
            self.ekf.x[2]   # theta orientation
        ]
        
        # Update robot state with the new estimates
        self.robot_state.update(
            x=estimated_pose[0],
            y=estimated_pose[1],
            theta=estimated_pose[2],
            velocity=encoder_data['velocity'],
            angular_velocity=gyro[2],
            lidar_scan=lidar_data
        )
        
        # Update SLAM
        dt_seconds = self.timestep / 1000.0  # Get time step in seconds

        # Create the pose_change tuple
        pose_change = (
            encoder_data['distance'] * 1000,  # dxy_mm (Convert to mm)
            math.degrees(encoder_data['dtheta']),  # dtheta_degrees
            dt_seconds  # dt_seconds
        )

        # Update SLAM with the correct parameter order
        self.slam.update(
            lidar_data,  # scans_mm (first parameter)
            pose_change,  # pose_change tuple (second parameter)
            self.lidar_adapter.get_angles()  # scan_angles_degrees (third parameter)
        )

    def set_motor_speeds(self, left_speed, right_speed):
        """Set the motor speeds"""
        # Apply PID control for more accurate speed control
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def plan_path(self, target_x, target_y):
        """Plan a path to the target position"""
        # Get current map from SLAM
        obstacle_map = self.slam.get_map()
    
        # Get current position
        start = (self.robot_state.x, self.robot_state.y)
        goal = (target_x, target_y)
    
        # Create a new RRTStar instance with the current map and positions
        self.path_planner = RRTStar(
            map_array=obstacle_map, 
            start=start, 
            goal=goal
        )
    
        # Plan the path (assuming plan() takes no arguments since they're already provided in constructor)
        self.path = self.path_planner.plan()
    
        self.current_path_index = 0
        self.target_position = goal

    def follow_path(self):
        """Follow the planned path"""
        if not self.path or self.current_path_index >= len(self.path):
            # No path or at the end of the path
            self.set_motor_speeds(0, 0)
            return
        
        # Get next waypoint
        waypoint = self.path[self.current_path_index]
        
        # Calculate control outputs using the path follower
        linear_vel, angular_vel = self.path_follower.compute_velocity(
            self.robot_state.x,
            self.robot_state.y,
            self.robot_state.theta,
            waypoint[0],
            waypoint[1]
        )
        
        # Convert to wheel velocities
        left_speed, right_speed = self.convert_to_wheel_speeds(linear_vel, angular_vel)
        
        # Set motor speeds
        self.set_motor_speeds(left_speed, right_speed)
        
        # Check if we've reached the current waypoint
        distance_to_waypoint = math.sqrt(
            (self.robot_state.x - waypoint[0])**2 + 
            (self.robot_state.y - waypoint[1])**2
        )
        
        if distance_to_waypoint < self.config.WAYPOINT_THRESHOLD:
            self.current_path_index += 1

    def convert_to_wheel_speeds(self, linear_vel, angular_vel):
        """Convert linear and angular velocity to wheel speeds"""
        # Calculate wheel speeds based on differential drive kinematics
        left_speed = (linear_vel - angular_vel * self.config.WHEEL_DISTANCE / 2) / self.config.WHEEL_RADIUS
        right_speed = (linear_vel + angular_vel * self.config.WHEEL_DISTANCE / 2) / self.config.WHEEL_RADIUS
        
        # Clamp to maximum speed
        max_speed = self.config.MAX_MOTOR_SPEED
        left_speed = max(min(left_speed, max_speed), -max_speed)
        right_speed = max(min(right_speed, max_speed), -max_speed)
        
        return left_speed, right_speed

    def visualize(self):
        """Visualize robot state and sensor data"""
        if self.display:
            # Implement visualization
            pass

    def run(self):
        """Main control loop"""
        print("[INFO] Two Wheel Robot Controller initialized")
        
        # Initialize display if available
        display = None
        try:
            display = setup_display(self.robot, 
                                self.config.DISPLAY_WIDTH, 
                                self.config.DISPLAY_HEIGHT)
        except Exception as e:
            print(f"[WARNING] Display initialization failed: {e}")
        # Main control loop
        while self.robot.step(self.timestep) != -1:
            # Update robot state
            self.update_robot_state()
            
            # Example: Follow a predefined path
            if not self.path:
                # Plan a path to a target
                self.plan_path(5.0, 5.0)
            
            # Follow the path
            self.follow_path()
            
            # Visualize
            self.visualize()
            
            # Log data
            log_data(self.robot_state, self.slam)

# Main function
def main():
    controller = TwoWheelController()
    controller.run()

if __name__ == "__main__":
    main()