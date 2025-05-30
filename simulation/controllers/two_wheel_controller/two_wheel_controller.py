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
class DummyScan:
    def __init__(self, size):
        self.size = size
        # Create reasonable defaults for all required attributes
        num_points = 100  # Number of scan points
        
        # Create distance array (all points at 1000mm by default)
        self.distances_mm = np.ones(num_points) * 1000
        
        # Create angle arrays (full 360 degrees)
        self.angles_rad = np.linspace(0, 2*np.pi, num_points)
        self.angles_deg = np.degrees(self.angles_rad).tolist()  # IMPORTANT: angles in degrees
        
        # Calculate x,y coordinates
        self.xs_mm = np.cos(self.angles_rad) * self.distances_mm
        self.ys_mm = np.sin(self.angles_rad) * self.distances_mm
    
    def update(self, scans_mm, hole_width_mm, *args, **kwargs):
        """Update scan with new distance readings"""
        if scans_mm is not None and len(scans_mm) > 0:
            # Create array from the input scans
            self.distances_mm = np.array(scans_mm)
            
            # If angles don't match the number of scan points, regenerate them
            if len(self.angles_rad) != len(scans_mm):
                self.angles_rad = np.linspace(0, 2*np.pi, len(scans_mm))
                self.angles_deg = np.degrees(self.angles_rad).tolist()
            
            # Update x,y coordinates
            self.xs_mm = np.cos(self.angles_rad) * self.distances_mm
            self.ys_mm = np.sin(self.angles_rad) * self.distances_mm
            
        return self.distances_mm        
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
        """Initialize SLAM components for localization and mapping"""
        
        class MapWrapper:
            def __init__(self, map_array):
                # Existing initialization code...
                self.map_array = map_array
                # Add map parameters
                self.map_size_pixels = 100  # Size in pixels (same as your initialized 100x100 array)
                self.map_size_meters = 10   # Size in meters (from your SLAM initialization)
            
            # Existing methods...
            
            def world_to_map(self, world_x_mm, world_y_mm):
                """Convert world coordinates (mm) to map pixel coordinates"""
                # Check for NaN inputs first
                if np.isnan(world_x_mm) or np.isnan(world_y_mm):
                    # Return center of map for NaN coordinates
                    return self.map_size_pixels // 2, self.map_size_pixels // 2
                
                # Convert mm to meters first
                world_x_m = world_x_mm / 1000.0
                world_y_m = world_y_mm / 1000.0
                
                # Calculate scaling factor (guard against division by zero)
                if self.map_size_meters == 0:
                    scale = 1.0  # Default scale if map_size_meters is zero
                else:
                    scale = self.map_size_pixels / self.map_size_meters
                
                # Convert to pixel coordinates
                # Center the origin in the middle of the map
                map_center = self.map_size_pixels / 2
                
                # Guard against NaN results
                try:
                    pixel_x = int(map_center + (world_x_m * scale))
                    pixel_y = int(map_center - (world_y_m * scale))  # Y is flipped in image coordinates
                except (ValueError, OverflowError):
                    # Fall back to center if calculation fails
                    return self.map_size_pixels // 2, self.map_size_pixels // 2
                
                # Ensure within map bounds
                pixel_x = max(0, min(self.map_size_pixels-1, pixel_x))
                pixel_y = max(0, min(self.map_size_pixels-1, pixel_y))
                
                return pixel_x, pixel_y
                
            def get_map(self):
                """Return the map as a 2D numpy array"""
                # Ensure returned map is always a valid 2D numpy array
                if not isinstance(self.map_array, np.ndarray) or len(self.map_array.shape) != 2:
                    print("[WARNING] Map not in correct format, returning empty map")
                    return np.zeros((100, 100), dtype=np.uint8)
                return self.map_array
            
            def update(self, scan, new_position, should_update=True, *args, **kwargs):
                """Update map with new scan data"""
                if not should_update:
                    return self.map_array
                
                # Just a placeholder - actual map updating would go here
                print("[INFO] Map update called (placeholder)")
                return self.map_array
        
        class DummyScan:
            def __init__(self, size):
                self.size = size
                # Create all required attributes for SLAM
                num_points = size
                
                # Distance array (initially 1000mm for all points)
                self.distances_mm = np.ones(num_points) * 1000
                
                # Angle arrays (full 360 degrees coverage)
                self.angles_rad = np.linspace(0, 2*np.pi, num_points)
                self.angles_deg = np.degrees(self.angles_rad).tolist()  # Required by SLAM
                
                # Cartesian coordinates
                self.xs_mm = np.cos(self.angles_rad) * self.distances_mm
                self.ys_mm = np.sin(self.angles_rad) * self.distances_mm
            
            def update(self, scans_mm, hole_width_mm, *args, **kwargs):
                """Update scan with new distance readings"""
                if scans_mm is not None and len(scans_mm) > 0:
                    # Create array from the input scans
                    self.distances_mm = np.array(scans_mm)
                    
                    # Ensure angles match scan points
                    if len(self.angles_rad) != len(scans_mm):
                        self.angles_rad = np.linspace(0, 2*np.pi, len(scans_mm))
                        self.angles_deg = np.degrees(self.angles_rad).tolist()
                    
                    # Update x,y coordinates
                    self.xs_mm = np.cos(self.angles_rad) * self.distances_mm
                    self.ys_mm = np.sin(self.angles_rad) * self.distances_mm
                    
                return self.distances_mm
        
        # Create laser parameters object
        class LaserParams:
            def __init__(self):
                self.offset_mm = 0  # Assume laser is at center of robot
        
        laser = LaserParams()
        
        # Get LiDAR resolution or use default
        if hasattr(self, 'lidar') and hasattr(self.lidar, 'getHorizontalResolution'):
            lidar_resolution = self.lidar.getHorizontalResolution()
        else:
            lidar_resolution = 100  # Default resolution
        
        # Create scan objects with proper resolution
        scan_for_distance = DummyScan(lidar_resolution)
        scan_for_mapbuild = DummyScan(lidar_resolution)
        
        # Create initial empty map (100x100)
        map_array = np.zeros((100, 100), dtype=np.uint8)
        map_wrapper = MapWrapper(map_array)
        
        try:
            # Initialize SLAM with all required components
            self.slam = ParticleFilterSLAM(
                laser=laser,
                map_size_pixels=100,    # 100x100 pixel map
                map_size_meters=10,     # 10x10 meter map
                map_quality=50,         # Default quality
                hole_width_mm=600       # Default hole width
            )
            
            # Set required objects on SLAM instance
            self.slam.scan_for_mapbuild = scan_for_mapbuild
            self.slam.scan_for_distance = scan_for_distance
            self.slam.map = map_wrapper
            
            print("[INFO] SLAM initialized successfully")
        except Exception as e:
            print(f"[ERROR] Failed to initialize SLAM: {str(e)}")
            raise
        

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
        if hasattr(self.ekf.x[2], 'shape') and self.ekf.x[2].shape:
            # If it's an array with dimensions
            if len(self.ekf.x[2].shape) > 0 and self.ekf.x[2].shape[0] > 1:
                # If it's a multi-element array, just take the first element
                theta = float(self.ekf.x[2][0])
            else:
                # It's a single element array
                theta = float(self.ekf.x[2])
        elif hasattr(self.ekf.x[2], 'item'):
            # If it has an item() method but no shape, try using item()
            try:
                theta = float(self.ekf.x[2].item())
            except ValueError:
                # If item() fails, use direct conversion
                theta = float(self.ekf.x[2])
        else:
            # If not an array, convert directly
            theta = float(self.ekf.x[2])       

        


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

    
    def get_position(self):
        """Get the current position of the robot (x, y, theta)"""
        # Extract scalar values from any potential NumPy arrays
        if hasattr(self.robot_state.x, 'shape'):
            # Handle arrays of any size
            x = float(self.robot_state.x[0]) if len(self.robot_state.x.shape) > 0 else float(self.robot_state.x)
            y = float(self.robot_state.y[0]) if len(self.robot_state.y.shape) > 0 else float(self.robot_state.y)
            theta = float(self.robot_state.theta[0]) if len(self.robot_state.theta.shape) > 0 else float(self.robot_state.theta)
        else:
            # Not a NumPy array
            x = float(self.robot_state.x)
            y = float(self.robot_state.y)
            theta = float(self.robot_state.theta)
        
        return (x, y, theta)
            
    
    def plan_path(self, goal_x, goal_y):
        # Get current position
        current_x, current_y = self.get_position()[:2]  # Extract just x, y
    
        # Create occupancy grid from SLAM map
        occupancy_grid = self.slam.map.get_map()  # Assuming this returns a 2D numpy array
    
        # Initialize path planner
        self.path_planner = RRTStar(
            map_array=occupancy_grid,
            start=(current_x, current_y),  # Make sure these are simple tuples or values
            goal=(goal_x, goal_y),
            max_iterations=1000,
            step_size=0.1,
            search_radius=0.5
        )

        # Plan the path (assuming plan() takes no arguments since they're already provided in constructor)
        self.path = self.path_planner.plan()
    
        self.current_path_index = 0
        self.target_position = (goal_x, goal_y)

    def follow_path(self):
        """Follow the planned path"""
        if not self.path or self.current_path_index >= len(self.path):
            # No path or at the end of the path
            self.set_motor_speeds(0, 0)
            return
        
        # Get next waypoint
        waypoint = self.path[self.current_path_index]
        
        # Get current position and convert to scalar values
        current_pos = self.get_position()
        current_x = float(current_pos[0]) if hasattr(current_pos[0], 'shape') else float(current_pos[0])
        current_y = float(current_pos[1]) if hasattr(current_pos[1], 'shape') else float(current_pos[1])
        current_theta = float(current_pos[2]) if hasattr(current_pos[2], 'shape') else float(current_pos[2])
        
        # Ensure waypoint coordinates are scalar values
        waypoint_x = float(waypoint[0]) if hasattr(waypoint[0], 'shape') else float(waypoint[0])
        waypoint_y = float(waypoint[1]) if hasattr(waypoint[1], 'shape') else float(waypoint[1])
        
        # Calculate distance to waypoint using scalar values
        distance_to_waypoint = math.sqrt(
            (waypoint_x - current_x)**2 + (waypoint_y - current_y)**2
        )
        
        # Calculate control outputs
        linear_vel, angular_vel = self.path_follower.compute_velocity(
            current_x, current_y, current_theta,
            waypoint_x, waypoint_y
        )
        
        # Set motor speeds
        self.set_motor_speeds(linear_vel, angular_vel)
        
        # Check if we've reached the waypoint
        if distance_to_waypoint < 0.1:  # 10cm tolerance
            self.current_path_index += 1
            print(f"Reached waypoint {self.current_path_index-1}, moving to next waypoint")

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
    def log_data(self, display=None):
        """Custom method to log data without causing errors with SLAM"""
        try:
            # Safely extract x coordinate
            if hasattr(self.robot_state.x, 'shape') and self.robot_state.x.shape:
                # If it's an array with dimensions, take first element
                robot_x = float(self.robot_state.x[0])
            else:
                # Otherwise convert directly
                robot_x = float(self.robot_state.x)
                
            # Safely extract y coordinate
            if hasattr(self.robot_state.y, 'shape') and self.robot_state.y.shape:
                robot_y = float(self.robot_state.y[0])
            else:
                robot_y = float(self.robot_state.y)
                
            # Safely extract theta/orientation
            if hasattr(self.robot_state.theta, 'shape') and self.robot_state.theta.shape:
                robot_theta = float(self.robot_state.theta[0])
            else:
                robot_theta = float(self.robot_state.theta)
                
            # Log information
            print(f"Robot position: ({robot_x:.2f}, {robot_y:.2f}, {robot_theta:.2f})")
        except Exception as e:
            print(f"[WARNING] Error logging position data: {str(e)}")
        
        # If you need to visualize on display, add that code here
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
            self.log_data(display)

# Main function
def main():
    controller = TwoWheelController()
    controller.run()

if __name__ == "__main__":
    main()