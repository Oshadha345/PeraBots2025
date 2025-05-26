### Project Structure
```
/software
    /core
        __init__.py
        robot.py
    /filter
        __init__.py
        kalman_filter.py
    /motor_control
        __init__.py
        motor.py
    /navigation
        __init__.py
        path_planning.py
    /pera_slam
        __init__.py
        slam.py
    /sensors
        __init__.py
        lidar.py
        imu.py
    main.py
```

### Example Code

#### core/robot.py
```python
class Robot:
    def __init__(self, motor, sensors):
        self.motor = motor
        self.sensors = sensors

    def move(self, speed, rotation):
        self.motor.set_speed(speed, rotation)

    def get_sensor_data(self):
        return {
            'lidar': self.sensors.lidar.get_data(),
            'imu': self.sensors.imu.get_data()
        }
```

#### motor_control/motor.py
```python
class Motor:
    def __init__(self):
        # Initialize motor parameters
        pass

    def set_speed(self, speed, rotation):
        # Code to set motor speed and rotation
        pass
```

#### sensors/lidar.py
```python
class Lidar:
    def __init__(self):
        # Initialize LIDAR parameters
        pass

    def get_data(self):
        # Code to get LIDAR data
        return []
```

#### sensors/imu.py
```python
class IMU:
    def __init__(self):
        # Initialize IMU parameters
        pass

    def get_data(self):
        # Code to get IMU data
        return {}
```

#### main.py
```python
from core.robot import Robot
from motor_control.motor import Motor
from sensors.lidar import Lidar
from sensors.imu import IMU

def main():
    # Initialize components
    motor = Motor()
    lidar = Lidar()
    imu = IMU()

    # Create robot instance
    robot = Robot(motor, {'lidar': lidar, 'imu': imu})

    # Main loop
    while True:
        # Get sensor data
        sensor_data = robot.get_sensor_data()
        print("Sensor Data:", sensor_data)

        # Example movement command
        robot.move(speed=1.0, rotation=0.5)

if __name__ == "__main__":
    main()
```

### Integration with Webots
To integrate this with Webots, you would typically use the Webots API to control the robot and read sensor data. Here's a basic example of how you might modify the `main.py` to work with Webots:

```python
from controller import Robot
from core.robot import Robot
from motor_control.motor import Motor
from sensors.lidar import Lidar
from sensors.imu import IMU

def main():
    # Create the Robot instance from Webots
    webots_robot = Robot()
    timestep = int(webots_robot.getBasicTimeStep())

    # Initialize components
    motor = Motor(webots_robot)
    lidar = Lidar(webots_robot)
    imu = IMU(webots_robot)

    # Create robot instance
    robot = Robot(motor, {'lidar': lidar, 'imu': imu})

    # Main loop
    while webots_robot.step(timestep) != -1:
        # Get sensor data
        sensor_data = robot.get_sensor_data()
        print("Sensor Data:", sensor_data)

        # Example movement command
        robot.move(speed=1.0, rotation=0.5)

if __name__ == "__main__":
    main()
```

### Notes
1. **Webots API**: You will need to implement the methods in your `Motor`, `Lidar`, and `IMU` classes to interact with the Webots API.
2. **Sensor Data**: Ensure that your `get_data` methods in the sensor classes return the correct data format expected by your application.
3. **Control Logic**: You may want to implement more sophisticated control logic in the main loop, such as obstacle avoidance or path planning using the data from LIDAR and IMU.
4. **Testing**: Test each module independently before integrating them into the main application to ensure they work as expected.

This is a basic framework to get you started. You can expand upon it based on your specific requirements and the functionality of your existing modules.