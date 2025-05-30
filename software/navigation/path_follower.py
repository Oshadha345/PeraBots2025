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
        
    def follow_path(self, path: List[Tuple[float, float]]):
        """
        Set a new path to follow.
        
        Args:
            path: List of (x, y) waypoints to follow
        """
        if len(path) < 2:
            print("Path too short, need at least 2 points")
            return
            
        # Smooth the path
        path_smoother = PathSmooth(path)
        self.current_path = path_smoother.smooth(method="shortcut")
        
        # Configure the pure pursuit controller
        self.pure_pursuit.set_path(self.current_path)
        self.is_following_path = True
        
        print(f"Starting to follow path with {len(self.current_path)} points")
        
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