"""
Wall Following Controller for Two-Wheeled Robot
This controller allows the robot to follow the inner wall while avoiding obstacles.
"""
from controller import Robot
import sys
import os
import numpy as np

# Add the src directory to the Python path for adapter imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'src'))
# Add software directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'software'))
from adapters.webots_lidar_adapter import WebotsLidarAdapter
from adapters.webots_imu_adapter import WebotsIMUAdapter
from adapters.webots_encoder_adapter import WebotsEncoderAdapter
from core.robot_state import RobotState
from filters.complementary import ComplementaryFilter
from motor_control.pid_controller import PID
from config.simulation_config import SimulationConfig

class WallFollowingController:
    def __init__(self):
        self.robot = Robot()
        self.config = SimulationConfig()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.lidar_adapter = WebotsLidarAdapter(self.robot.getDevice('lidar'), self.config)
        self.imu_adapter = WebotsIMUAdapter(self.robot.getDevice('inertial_unit'), 
                                             self.robot.getDevice('accelerometer'), 
                                             self.robot.getDevice('gyro'), 
                                             self.config)
        self.encoder_adapter = WebotsEncoderAdapter(
            self.robot.getDevice('left_wheel_sensor'), 
            self.robot.getDevice('right_wheel_sensor'), 
            self.config.WHEEL_RADIUS, 
            self.config.WHEEL_DISTANCE
        )
        
        self.robot_state = RobotState()
        self.comp_filter = ComplementaryFilter(alpha=0.98)
        self.left_pid = PID(self.config.PID_KP, self.config.PID_KI, self.config.PID_KD)
        self.right_pid = PID(self.config.PID_KP, self.config.PID_KI, self.config.PID_KD)

    def update_robot_state(self):
        lidar_data = self.lidar_adapter.get_scan()
        imu_data = self.imu_adapter.get_data()
        encoder_data = self.encoder_adapter.get_data()
        
        accel = imu_data['accel']
        gyro = imu_data['gyro']
        orientation = imu_data['orientation']
        
        filtered_orientation = self.comp_filter.update(accel, gyro, orientation)
        
        self.robot_state.update(
            x=self.robot_state.x,
            y=self.robot_state.y,
            theta=filtered_orientation[2],
            velocity=encoder_data['velocity'],
            angular_velocity=gyro[2],
            lidar_scan=lidar_data
        )

    def follow_wall(self):
        distances = list(self.lidar_adapter.get_distances())
        closest_distance = min(distances)
        closest_index = distances.index(closest_distance)
        
        # Use LIDAR indices for left, right, and front
        n = len(distances)
        left_distance = distances[int(n * 0.25)]
        right_distance = distances[int(n * 0.75)]
        front_distance = distances[int(n * 0.5)]
        
        if front_distance < self.config.WALL_DISTANCE_THRESHOLD:
            self.avoid_obstacle(left_distance, right_distance)
        else:
            self.set_motor_speeds(self.config.DEFAULT_SPEED, self.config.DEFAULT_SPEED)

    def avoid_obstacle(self, left_distance, right_distance):
        # Turn away from the closer wall/obstacle
        if left_distance < right_distance:
            self.set_motor_speeds(-self.config.DEFAULT_SPEED, self.config.DEFAULT_SPEED)  # Turn right
        else:
            self.set_motor_speeds(self.config.DEFAULT_SPEED, -self.config.DEFAULT_SPEED)  # Turn left

    def set_motor_speeds(self, left_speed, right_speed):
        self.robot.getDevice('left_wheel_motor').setVelocity(left_speed)
        self.robot.getDevice('right_wheel_motor').setVelocity(right_speed)

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.update_robot_state()
            self.follow_wall()

def main():
    controller = WallFollowingController()
    controller.run()

if __name__ == "__main__":
    main()