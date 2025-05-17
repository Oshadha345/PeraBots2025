"""
Main Entry Point for Robot Brain
"""

from sensors import imu, lidar
from motor_control import pwm_driver
from filters import kalman
from core import robot_state

def main():
    print("[*] Booting Robot...")
    imu_data = imu.read()
    filtered_data = kalman.apply(imu_data)

    while True:
        state = robot_state.update(filtered_data)
        pwm_driver.drive(state.motor_command)

if __name__ == "__main__":
    main()
