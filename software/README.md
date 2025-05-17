# 🧠 Software Architecture (Raspberry Pi 4)

This folder contains all the high-level and low-level software modules that run on the Raspberry Pi 4 for our robotics system.

---

## 📦 Modules

| Module          | Description |
|-----------------|-------------|
| `slam/`         | SLAM algorithms like GMapping, occupancy grid mapping |
| `filters/`      | Sensor fusion techniques like Kalman and Complementary filters |
| `motor_control/`| Motor drivers, PID controllers, PWM output management |
| `sensors/`      | Interfaces for IMU, LIDAR, Camera, Ultrasonic sensors |
| `core/`         | Robot state manager, configuration, logging |
| `main.py`       | The entry point of the robot's runtime logic |

---

## 🧰 Raspberry Pi 4 Setup

1. Install dependencies:

```bash
sudo apt update
sudo apt install python3-pip
pip3 install -r requirements.txt
```

2. Enable I2C, SPI, and Serial via `raspi-config`.

3. To run the robot:

```python

python3 main.py

```

---

## 🔄 Modularity

Each module runs independently but communicates through shared objects and ROS-like design principles. For example:

- Sensor data is passed into robot_state through filters

- SLAM modules read from filtered sensor data

- Motor control module uses commands from robot_state.navigator

This enables easy debugging, hot-swapping modules, and team-based development.

---

## 🔗 External Libraries Used


- NumPy
- OpenCV
- PySerial
- Matplotlib (for visualization)
- `smbus2`, `RPi.GPIO` for hardware interfaces

---