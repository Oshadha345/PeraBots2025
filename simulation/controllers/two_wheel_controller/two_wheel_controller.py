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
# sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'software'))
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
from pera_slam import ParticleFilterSLAM, RPLidarA1, SLAMMapVisualizer
from navigation.path_follower import PathFollower
from path_planning import RRTStar

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

        # State machine for robot operation
        self.ROBOT_STATE_EXPLORE = 0
        self.ROBOT_STATE_RETURN_HOME = 1
        self.ROBOT_STATE_PLAN_PATH = 2
        self.ROBOT_STATE_FOLLOW_PATH = 3
        self.current_state = self.ROBOT_STATE_EXPLORE

        # Exploration parameters
        self.home_position = [0, 0]  # Will be set when robot starts
        self.exploration_start_time = None
        self.exploration_timeout = 180  # 3 minutes

        # Coverage tracking
        self.coverage_threshold = 0.75  # 75% map coverage
        self.prev_coverage = 0.0
        self.stall_counter = 0

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

        # Set more scan points if available
        if hasattr(self.lidar, 'getHorizontalResolution'):
            current_res = self.lidar.getHorizontalResolution()
            print(f"[INFO] Current LiDAR horizontal resolution: {current_res}")

        if hasattr(self.lidar, 'setHorizontalResolution'):
            try:
                self.lidar.setHorizontalResolution(360)  # Try to set 360 points
                print("[INFO] Set LiDAR horizontal resolution to 360")
            except Exception as e:
                print(f"[WARNING] Could not set LiDAR resolution: {e}")

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
        # Create adapter instances with proper configuration
        # Set higher resolution for LiDAR (more scan points)
        self.lidar.enablePointCloud()

        # Set proper horizontal resolution - higher number = more points
        if hasattr(self.lidar, 'setHorizontalResolution'):
            self.lidar.setHorizontalResolution(360)  # Get 360 points (1 degree resolution)

        # Now create the adapter with properly configured LiDAR
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

        class DummyScan:
            def __init__(self, size):
                self.size = size
                # Add distances_mm attribute with default values
                self.distances_mm = np.ones(size) * 1000  # Default 1000mm (1m) distance
                # Add angles_deg attribute
                self.angles_deg = np.linspace(-180, 180, size)

            def update(self, scans_mm, hole_width_mm, *args, **kwargs):
                # Minimal implementation that updates distances_mm
                if scans_mm is not None and len(scans_mm) > 0:
                    self.distances_mm = np.array(scans_mm)
                    # Regenerate angles if needed
                    if len(self.angles_deg) != len(self.distances_mm):
                        self.angles_deg = np.linspace(-180, 180, len(self.distances_mm))

        # Create and add the scan objects to the SLAM instance
        self.slam.scan_for_distance = DummyScan(self.config.MAP_SIZE_PIXELS)
        self.slam.scan_for_mapbuild = DummyScan(self.config.MAP_SIZE_PIXELS)

        class MapWrapper:
            def __init__(self, map_array):
                self.map_array = map_array
                # Size of the map (assuming it's square)
                self.size_pixels = map_array.shape[0]
                # Store parameters from config
                self.size_meters = 20.0  # This should match your MAP_SIZE_METERS in config
                # Add shape attribute to match NumPy array interface
                self.shape = map_array.shape

            def __getitem__(self, key):
                """Make the MapWrapper subscriptable by delegating to map_array"""
                return self.map_array[key]

            # Update the world_to_map method in the MapWrapper class

            def world_to_map(self, world_x, world_y):
                """Convert world coordinates (meters) to map pixel coordinates"""

                # Check for NaN values
                if np.isnan(world_x) or np.isnan(world_y):
                    print(f"[WARNING] Received NaN coordinates in world_to_map: ({world_x}, {world_y})")
                    # Return center of map as fallback
                    return self.size_pixels // 2, self.size_pixels // 2

                # Convert from world coordinates to pixel coordinates
                map_x = int((world_x + self.size_meters / 2) / self.size_meters * self.size_pixels)
                map_y = int((world_y + self.size_meters / 2) / self.size_meters * self.size_pixels)

                # Ensure coordinates are within map bounds
                map_x = max(0, min(map_x, self.size_pixels - 1))
                map_y = max(0, min(map_y, self.size_pixels - 1))

                return map_x, map_y

            def get_map(self):
                """Return the current map in probability format (0-1)"""
                # If map is stored in log-odds format, convert to probabilities
                if np.max(np.abs(self.map_array)) > 1.0:  # Map is in log-odds
                    return 1.0 - 1.0 / (1.0 + np.exp(self.map_array))
                return self.map_array

            def update(self, scan, position, quality=50, hole_width_mm=600, *args, **kwargs):
                """
                Update map based on lidar scan and robot position

                Args:
                    scan: Scan object with distances_mm attribute
                    position: Position object with x_mm, y_mm, theta_degrees attributes
                    quality: Update quality (0-255)
                    hole_width_mm: Width of obstacles in mm
                """
                try:
                    scan_length = len(scan.distances_mm)
                    print(f"[INFO] Updating map with {scan_length} scan points")

                    # Convert position to pixel coordinates
                    try:
                        # If position is a Position object
                        pos_x_mm = position.x_mm
                        pos_y_mm = position.y_mm
                        pos_theta_deg = position.theta_degrees
                    except AttributeError:
                        # If position is a tuple or other structure
                        pos_x_mm = position[0] * 1000  # Convert m to mm
                        pos_y_mm = position[1] * 1000
                        pos_theta_deg = math.degrees(position[2]) if len(position) > 2 else 0

                    # Check position for NaN values
                    if np.isnan(pos_x_mm) or np.isnan(pos_y_mm) or np.isnan(pos_theta_deg):
                        print(f"[WARNING] Invalid position detected: ({pos_x_mm}, {pos_y_mm}, {pos_theta_deg})")
                        return False

                    # Convert robot position to map pixel coordinates
                    pos_pix_x = int((pos_x_mm / 1000.0 + self.size_meters / 2) / self.size_meters * self.size_pixels)
                    pos_pix_y = int((pos_y_mm / 1000.0 + self.size_meters / 2) / self.size_meters * self.size_pixels)

                    # Ensure robot position is within map bounds
                    if not (0 <= pos_pix_x < self.size_pixels and 0 <= pos_pix_y < self.size_pixels):
                        print(f"[WARNING] Robot position ({pos_pix_x}, {pos_pix_y}) is outside map bounds")
                        pos_pix_x = max(0, min(pos_pix_x, self.size_pixels - 1))
                        pos_pix_y = max(0, min(pos_pix_y, self.size_pixels - 1))

                    # Quality factor for map updates
                    quality_factor = quality / 255.0
                    log_odds_update = np.log(quality_factor / (1.0 - quality_factor))

                    # Ensure map is in log-odds format
                    if np.max(np.abs(self.map_array)) <= 1.0:  # Map is in probabilities
                        # Convert to log-odds format for updating
                        valid_probs = np.clip(self.map_array, 0.001, 0.999)
                        self.map_array = np.log(valid_probs / (1.0 - valid_probs))

                    # Get scan angles - either from scan object or generate default angles
                    try:
                        angles_deg = scan.angles_deg
                    except AttributeError:
                        # Generate default angles if not available
                        angles_deg = np.linspace(-180, 180, scan_length)

                    # Update map for each valid scan point
                    valid_updates = 0
                    for i in range(scan_length):
                        try:
                            dist_mm = scan.distances_mm[i]

                            # Skip invalid readings more thoroughly
                            if (dist_mm <= 0 or dist_mm == float('inf') or dist_mm == float('-inf')
                                    or np.isnan(dist_mm) or dist_mm > 10000):  # Skip readings over 10m too
                                continue

                            # Calculate endpoint in world coordinates
                            angle_rad = np.radians(pos_theta_deg + angles_deg[i])
                            dist_m = dist_mm / 1000.0  # convert to meters

                            endpoint_x_m = pos_x_mm / 1000.0 + np.cos(angle_rad) * dist_m
                            endpoint_y_m = pos_y_mm / 1000.0 + np.sin(angle_rad) * dist_m

                            # Skip if calculated endpoint is invalid
                            if np.isnan(endpoint_x_m) or np.isnan(endpoint_y_m):
                                continue

                            # Convert endpoint to pixel coordinates
                            endpoint_pix_x = int(
                                (endpoint_x_m + self.size_meters / 2) / self.size_meters * self.size_pixels)
                            endpoint_pix_y = int(
                                (endpoint_y_m + self.size_meters / 2) / self.size_meters * self.size_pixels)

                            # Skip if out of bounds
                            if (endpoint_pix_x < 0 or endpoint_pix_x >= self.size_pixels or
                                    endpoint_pix_y < 0 or endpoint_pix_y >= self.size_pixels):
                                continue

                            # Use Bresenham's line algorithm to trace the ray
                            points = self._bresenham(pos_pix_x, pos_pix_y, endpoint_pix_x, endpoint_pix_y)

                            # Mark path as free space
                            for j in range(len(points) - 1):
                                x, y = points[j]
                                if 0 <= x < self.size_pixels and 0 <= y < self.size_pixels:
                                    self.map_array[y, x] -= log_odds_update * 0.5  # Free space update

                            # Mark endpoint as occupied
                            if points and 0 <= points[-1][0] < self.size_pixels and 0 <= points[-1][
                                1] < self.size_pixels:
                                x, y = points[-1]
                                self.map_array[y, x] += log_odds_update  # Occupied space update
                                valid_updates += 1

                        except Exception as e:
                            print(f"[WARNING] Error processing scan point {i}: {e}")
                            continue

                    # Clip values to prevent numerical issues
                    np.clip(self.map_array, -100, 100, out=self.map_array)

                    print(f"[INFO] Map updated with {valid_updates} valid points")
                    return True

                except Exception as e:
                    print(f"[ERROR] Failed to update map: {e}")
                    return False

            def _bresenham(self, x0, y0, x1, y1):
                """Bresenham's line algorithm for tracing rays through grid"""
                points = []
                dx = abs(x1 - x0)
                dy = abs(y1 - y0)
                sx = 1 if x0 < x1 else -1
                sy = 1 if y0 < y1 else -1
                err = dx - dy

                while True:
                    points.append((x0, y0))
                    if x0 == x1 and y0 == y1:
                        break
                    e2 = 2 * err
                    if e2 > -dy:
                        err -= dy
                        x0 += sx
                    if e2 < dx:
                        err += dx
                        y0 += sy

                return points

        # Replace the NumPy array with the wrapper object
        self.slam.map = MapWrapper(self.slam.map)

    def _init_navigation(self):
        """Initialize path planning and following modules"""
        # Path planning
        self.path_planner = None

        # Path following
        self.path_follower = PathFollower(motor_controller=self)

    def update_robot_state(self):
        """Update robot state with current sensor readings and estimated pose"""
        # Get sensor readings
        lidar_data_obj = self.lidar_adapter.get_scan() # This is a ScanData object
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
        # Ensure self.ekf.x[2] is a scalar before using in np.cos/np.sin
        current_theta_from_ekf = self.get_scalar_value(self.ekf.x[2])

        # Update state transition matrix for non-linear motion
        self.ekf.F[0, 3] = dt * np.cos(current_theta_from_ekf)  # x += v*dt*cos(theta)
        self.ekf.F[1, 3] = dt * np.sin(current_theta_from_ekf)  # y += v*dt*sin(theta)
        self.ekf.F[2, 4] = dt  # theta += omega*dt

        # Define measurement function and its Jacobian
        def Hx(x_state): # Renamed x to x_state to avoid conflict
            """Measurement function - converts state to measurement"""
            # Measurement vector: [velocity, angular_velocity, orientation]
            return np.array([
                x_state[3],  # velocity
                x_state[4],  # angular velocity
                x_state[2]  # orientation (theta)
            ])

        def HJacobian(x_state): # Renamed x to x_state
            """Compute Jacobian of the measurement function"""
            H = np.zeros((3, 5))
            H[0, 3] = 1.0
            H[1, 4] = 1.0
            H[2, 2] = 1.0
            return H

        # Step 2: Perform prediction step (time update)
        self.ekf.predict()

        # Step 3: Create measurement vector from sensors
        # Ensure velocities are scalars
        left_vel_scalar = self.get_scalar_value(encoder_data['left_velocity'])
        right_vel_scalar = self.get_scalar_value(encoder_data['right_velocity'])
        
        z = np.array([
            left_vel_scalar, 
            right_vel_scalar,
            self.get_scalar_value(filtered_orientation[2])  # Yaw/heading
        ])

        # Step 4: Perform correction step (measurement update) with required functions
        self.ekf.update(z, HJacobian, Hx)

        # Step 5: Extract the estimated state
        # This is where estimated_pose is assigned
        estimated_pose = [
            self.get_scalar_value(self.ekf.x[0]),  # x position
            self.get_scalar_value(self.ekf.x[1]),  # y position
            self.get_scalar_value(self.ekf.x[2])  # theta orientation
        ]

        # NOW it's safe to update robot_state and SLAM using estimated_pose

        # Update robot state with the new estimates
        self.robot_state.update(
            x=estimated_pose[0],
            y=estimated_pose[1],
            theta=estimated_pose[2],
            velocity=self.get_scalar_value(encoder_data['velocity']),
            angular_velocity=self.get_scalar_value(gyro[2]),
            lidar_scan=lidar_data_obj
        )

        # Update SLAM
        dt_seconds = self.timestep / 1000.0  # Get time step in seconds

        # Create the pose_change tuple
        pose_change = (
            self.get_scalar_value(encoder_data['distance']) * 1000,  # dxy_mm (Convert to mm)
            math.degrees(self.get_scalar_value(encoder_data['dtheta'])),  # dtheta_degrees
            dt_seconds  # dt_seconds
        )
        
        if hasattr(lidar_data_obj, 'distances_mm') and hasattr(lidar_data_obj, 'angles_deg'):
            self.slam.update(
                lidar_data_obj.distances_mm,
                pose_change,
                lidar_data_obj.angles_deg
            )
        else:
            print("[WARNING] SLAM update skipped: lidar_data_obj missing attributes")

    def set_motor_speeds(self, left_speed, right_speed):
        """Set the motor speeds"""
        # Apply PID control for more accurate speed control
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def plan_path(self, target_x, target_y):
        """Plan a path to the target position"""
        # Get current map from SLAM
        obstacle_map = self.slam.get_map()

        # Get current position with safer conversion to handle multi-element arrays
        if isinstance(self.robot_state.x, np.ndarray):
            if self.robot_state.x.size == 1:
                start_x = float(self.robot_state.x.item())
            else:
                # Use the first element if it's a multi-element array
                start_x = float(self.robot_state.x[0])
        else:
            start_x = float(self.robot_state.x)

        if isinstance(self.robot_state.y, np.ndarray):
            if self.robot_state.y.size == 1:
                start_y = float(self.robot_state.y.item())
            else:
                # Use the first element if it's a multi-element array
                start_y = float(self.robot_state.y[0])
        else:
            start_y = float(self.robot_state.y)

        start = (start_x, start_y)
        goal = (target_x, target_y)

        print(f"Planning path from {start} to {goal}")

        # Create a new RRTStar instance with the current map and positions
        self.path_planner = RRTStar(
            map_array=obstacle_map,
            start=start,
            goal=goal
        )

        # Plan the path
        self.path = self.path_planner.plan()

        if self.path:
            print(f"Successfully planned path with {len(self.path)} waypoints")
        else:
            print("Failed to plan path")

        self.current_path_index = 0
        self.target_position = goal

    def get_scalar_value(self, array_or_scalar):
        """Safely convert NumPy arrays to scalar values"""
        if isinstance(array_or_scalar, np.ndarray):
            if array_or_scalar.size == 1:
                return float(array_or_scalar.item())
            elif array_or_scalar.size > 1:
                return float(array_or_scalar[0])
            else:
                return 0.0  # Empty array
        else:
            return float(array_or_scalar)

    def follow_path(self):
        """Follow the planned path"""
        if not self.path or self.current_path_index >= len(self.path):
            # No path or at the end of the path
            self.set_motor_speeds(0, 0)
            return

        # Get next waypoint
        waypoint = self.path[self.current_path_index]

        # Get current position safely
        current_x = self.get_scalar_value(self.robot_state.x)
        current_y = self.get_scalar_value(self.robot_state.y)
        current_theta = self.get_scalar_value(self.robot_state.theta)

        # Calculate control outputs using the path follower
        linear_vel, angular_vel = self.path_follower.compute_velocity(
            current_x,
            current_y,
            current_theta,
            waypoint[0],
            waypoint[1]
        )

        # Convert to wheel velocities
        left_speed, right_speed = self.convert_to_wheel_speeds(linear_vel, angular_vel)

        # Set motor speeds
        self.set_motor_speeds(left_speed, right_speed)

        # Check if we've reached the current waypoint
        distance_to_waypoint = math.sqrt(
            (current_x - waypoint[0]) ** 2 +
            (current_y - waypoint[1]) ** 2
        )

        if distance_to_waypoint < self.config.WAYPOINT_THRESHOLD:
            print(f"Reached waypoint {self.current_path_index}, moving to next waypoint")
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

    # Add these methods before the run method

    def return_to_home(self):
        """Navigate back to starting position"""
        # Simple implementation - just plan a path to home if we don't have one
        if not self.path:
            print("[INFO] Planning path back to home")
            self.plan_path(self.home_position[0], self.home_position[1])

        # Follow the existing path
        self.follow_path()

        # Check if we're back home
        current_x = self.get_scalar_value(self.robot_state.x)
        current_y = self.get_scalar_value(self.robot_state.y)

        distance_to_home = math.sqrt(
            (current_x - self.home_position[0]) ** 2 +
            (current_y - self.home_position[1]) ** 2
        )

        if distance_to_home < 0.3:  # Within 30cm of home
            print("[INFO] Successfully returned to home position")
            # Transition to planning state
            self.current_state = self.ROBOT_STATE_PLAN_PATH

    def plan_optimal_path(self):
        """Plan optimal path around inner wall (placeholder)"""
        print("[INFO] Planning optimal path around environment")

        # Simplified circular path for testing
        radius = 1.0
        num_points = 8
        path = []

        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            path.append((x, y))

        # Add the starting point at the end to complete the loop
        path.append((0, 0))

        self.path = path
        self.current_path_index = 0
        print(f"[INFO] Created circular path with {len(self.path)} waypoints")

        # Transition to path following state
        self.current_state = self.ROBOT_STATE_FOLLOW_PATH

    def follow_optimal_path(self):
        """Follow the planned optimal path"""
        # Use existing path following method
        self.follow_path()

        # Check if we've completed the path
        if self.current_path_index >= len(self.path):
            print("[INFO] Navigation complete!")
            # Stop the robot
            self.set_motor_speeds(0, 0)

    # the explore_environment method
    def explore_environment(self):
        """Execute exploration behavior to map the environment"""
        # Check if we've been exploring long enough
        current_time = self.robot.getTime()
        if current_time - self.exploration_start_time > 30:  # 30 seconds for testing
            print("[INFO] Exploration phase complete, transitioning to return home")
            self.current_state = self.ROBOT_STATE_RETURN_HOME
            return

        # Get LiDAR data for obstacle detection
        lidar_data_obj = self.robot_state.lidar_scan # lidar_data_obj is a ScanData object

        # Simple wall-following behavior
        # Find minimum distance in front, left and right - with better error handling
        # Access .distances_mm from the ScanData object
        if hasattr(lidar_data_obj, 'distances_mm'):
            lidar_distances = lidar_data_obj.distances_mm
            front_sector = [d for d in (lidar_distances[:30] + lidar_distances[-30:]) if d < float('inf') and d > 0]
            left_sector = [d for d in lidar_distances[30:90] if d < float('inf') and d > 0]
            right_sector = [d for d in lidar_distances[-90:-30] if d < float('inf') and d > 0]
        else:
            # Fallback if distances_mm is not available (should not happen with current adapter)
            print("[WARNING] lidar_data.distances_mm not found in explore_environment")
            front_sector, left_sector, right_sector = [1.0], [1.0], [1.0]

        front_dist = min(front_sector, default=1.0)  # Default to 1.0m if no valid readings
        left_dist = min(left_sector, default=1.0)
        right_dist = min(right_sector, default=1.0)

        # Debug info
        if int(self.robot.getTime()) % 5 == 0:  # Every 5 seconds exactly
            print(f"[DEBUG] Distances - Front: {front_dist:.2f}, Left: {left_dist:.2f}, Right: {right_dist:.2f}")

        # More cautious wall following logic
        if front_dist < 0.5:  # Obstacle ahead - back up and turn
            if front_dist < 0.2:  # Very close - back up
                self.set_motor_speeds(-0.3, -0.3)
                print("[DEBUG] Backing up from obstacle")
            else:  # Turn away from obstacle
                if left_dist > right_dist:
                    # Turn left
                    self.set_motor_speeds(0.0, 0.4)  # Sharper turn with one wheel stopped
                    print("[DEBUG] Turning left to avoid obstacle")
                else:
                    # Turn right
                    self.set_motor_speeds(0.4, 0.0)  # Sharper turn with one wheel stopped
                    print("[DEBUG] Turning right to avoid obstacle")
        elif right_dist < 0.3:  # Too close to right wall
            # Turn left slightly
            self.set_motor_speeds(0.2, 0.4)
        elif right_dist > 1.0:  # No wall on right
            # Turn right to find wall
            self.set_motor_speeds(0.4, 0.2)
        else:
            # Follow right wall at a good distance
            self.set_motor_speeds(0.4, 0.4)

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

        # Set home position and start time
        self.home_position = [0, 0]  # Assuming robot starts at origin
        self.exploration_start_time = self.robot.getTime()
        print(f"[INFO] Starting exploration phase at time {self.exploration_start_time}")

        # Main control loop
        while self.robot.step(self.timestep) != -1:
            # Update robot state
            self.update_robot_state()

            # Print debug info
            current_x = self.get_scalar_value(self.robot_state.x)
            current_y = self.get_scalar_value(self.robot_state.y)
            current_theta = self.get_scalar_value(self.robot_state.theta)

            # Debug output every 50 timesteps
            if self.robot.getTime() % 2 < 0.1:  # Roughly every 2 seconds
                print(f"Position: ({current_x:.2f}, {current_y:.2f}, {current_theta:.2f}), State: {self.current_state}")

            # Execute behavior based on current state
            if self.current_state == self.ROBOT_STATE_EXPLORE:
                self.explore_environment()
            elif self.current_state == self.ROBOT_STATE_RETURN_HOME:
                self.return_to_home()
            elif self.current_state == self.ROBOT_STATE_PLAN_PATH:
                self.plan_optimal_path()
            elif self.current_state == self.ROBOT_STATE_FOLLOW_PATH:
                self.follow_optimal_path()

            # Visualize
            self.visualize()

            # Log data
            # Log data with simple error handling
            try:
                # Add a dummy robot_state attribute if needed
                if not hasattr(self.slam, 'robot_state'):
                    self.slam.robot_state = self.robot_state
                log_data(self.robot_state, self.slam)
            except AttributeError as e:
                # Log error but continue execution
                print(f"[WARNING] Logging error: {e}")


# Main function
def main():
    controller = TwoWheelController()
    controller.run()


if __name__ == "__main__":
    main()
