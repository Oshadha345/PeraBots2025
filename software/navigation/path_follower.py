import time
import math
from typing import List, Tuple, Optional

# Import from your existing modules
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from motor_control.pid_controller import PID
from Path_planning.RRT_pathsmooth import PathSmooth
from navigation.pure_pursuit import PurePursuit

class PathFollower:
    """
    High-level path following controller that combines path planning,
    pure pursuit, and motor control.
    """
    
    def __init__(self, motor_controller, lookahead_distance=0.3):
        """
        Initialize the path follower.
        
        Args:
            motor_controller: Object that controls the robot's motors
            lookahead_distance: Distance to look ahead on the path (meters)
        """
        self.motor_controller = motor_controller
        self.pure_pursuit = PurePursuit(lookahead_distance=lookahead_distance)
        
        # PID controllers for linear and angular velocity
        self.linear_pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)
        self.angular_pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        
        # Path state
        self.current_path = []
        self.is_following_path = False
        
    def set_pose(self, x: float, y: float, heading: float):
        """Set the current robot pose."""
        self.x = x
        self.y = y
        self.heading = heading
        
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
        
        # Use path follower to get velocity commands
        linear_vel, angular_vel = self.path_follower.compute_velocity(
            current_x, current_y, current_theta, 
            waypoint_x, waypoint_y
        )
        
        # Set motor speeds based on computed velocities
        left_speed, right_speed = self.velocities_to_motor_commands(linear_vel, angular_vel)
        self.set_motor_speeds(left_speed, right_speed)
        
        # Check if we've reached the current waypoint (within tolerance)
        if distance_to_waypoint < 0.1:  # 10cm tolerance
            self.current_path_index += 1
            print(f"Reached waypoint {self.current_path_index-1}, moving to next waypoint")
            
    def update(self, x: float, y: float, heading: float, dt: float):
        """
        Update the path follower with the current robot pose.
        
        Args:
            x: Current x position
            y: Current y position
            heading: Current heading in radians
            dt: Time delta since last update
            
        Returns:
            is_complete: Whether the path following is complete
        """
        if not self.is_following_path:
            return True
            
        # Update robot state
        self.set_pose(x, y, heading)
        
        # Get velocity commands from pure pursuit
        linear_vel, angular_vel, path_completed = self.pure_pursuit.update(x, y, heading)
        
        if path_completed:
            self.stop()
            self.is_following_path = False
            return True
            
        # Convert velocities to motor commands
        left_speed, right_speed = self.velocities_to_motor_commands(linear_vel, angular_vel)
        
        # Send to motor controller
        self.motor_controller.set_motor_speeds(left_speed, right_speed)
        
        return False
    def compute_velocity(self, robot_x, robot_y, robot_theta, target_x, target_y):
        """
        Compute linear and angular velocities to reach a target point.
        
        Args:
            robot_x: Current robot x position
            robot_y: Current robot y position
            robot_theta: Current robot heading in radians
            target_x: Target x position
            target_y: Target y position
            
        Returns:
            (linear_vel, angular_vel): Velocity commands
        """
        # Convert potential NumPy arrays to scalar values
        robot_x = float(robot_x[0]) if hasattr(robot_x, 'shape') and robot_x.shape else float(robot_x)
        robot_y = float(robot_y[0]) if hasattr(robot_y, 'shape') and robot_y.shape else float(robot_y)
        robot_theta = float(robot_theta[0]) if hasattr(robot_theta, 'shape') and robot_theta.shape else float(robot_theta)
        target_x = float(target_x[0]) if hasattr(target_x, 'shape') and target_x.shape else float(target_x)
        target_y = float(target_y[0]) if hasattr(target_y, 'shape') and target_y.shape else float(target_y)
        # Calculate distance and angle to target
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate desired heading (angle to target)
        desired_heading = math.atan2(dy, dx)
        
        # Calculate heading error (normalize to -pi to pi)
        heading_error = desired_heading - robot_theta
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Simple proportional control
        # You can adjust these gains based on your robot's performance
        linear_vel = min(0.5, 0.3 * distance)  # Cap at 0.5 m/s
        angular_vel = 1.0 * heading_error      # Proportional gain of 1.0
        
        return linear_vel, angular_vel    
    def velocities_to_motor_commands(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """
        Convert linear and angular velocities to left and right wheel speeds.
        
        Args:
            linear_vel: Linear velocity in m/s
            angular_vel: Angular velocity in rad/s
            
        Returns:
            (left_speed, right_speed) in normalized units (-1.0 to 1.0)
        """
        # Robot parameters (these should be configured for your specific robot)
        wheel_base = 0.2  # Distance between wheels in meters
        
        # Calculate wheel speeds
        left_speed = linear_vel - (angular_vel * wheel_base / 2)
        right_speed = linear_vel + (angular_vel * wheel_base / 2)
        
        # Normalize to range [-1, 1]
        max_speed = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed = left_speed / max_speed
        right_speed = right_speed / max_speed
        
        return left_speed, right_speed
        
    def stop(self):
        """Stop the robot."""
        self.motor_controller.set_motor_speeds(0, 0)
        self.is_following_path = False
        print("Path following complete")