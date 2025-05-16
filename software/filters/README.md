# 🔄 Sensor Fusion Filters

Implements Kalman and Complementary filters for combining sensor data from IMU, GPS, and encoders.

## Files

- `kalman.py`: Extended Kalman Filter implementation
- `complementary.py`: Lightweight fusion for angle estimation

These are used to smooth noisy inputs before feeding them into higher-level modules like SLAM or motor control.
