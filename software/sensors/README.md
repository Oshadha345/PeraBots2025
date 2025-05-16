# 🧩 Sensor Interfaces

All drivers and wrappers for various sensors used in the bot.

## Files

- `imu.py`: MPU6050 / BNO055 using I2C
- `lidar.py`: RPLidar / LidarLite
- `camera.py`: OpenCV-based camera interface

These modules publish filtered data to the `core/robot_state`.
