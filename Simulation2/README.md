# Two-Wheel Wall Following Robot

This project implements a wall-following robot using a two-wheeled differential drive system. The robot is designed to maintain a specified distance from the inner wall while avoiding obstacles, creating an efficient path along the wall.

## Project Structure

```
two_wheel_wall_following
├── src
│   ├── wall_following_controller.py       # Main logic for wall-following behavior
│   ├── adapters
│   │   ├── webots_lidar_adapter.py        # Interfaces with the LiDAR sensor
│   │   ├── webots_imu_adapter.py          # Interfaces with the IMU sensor
│   │   └── webots_encoder_adapter.py      # Interfaces with wheel encoders
│   ├── core
│   │   └── robot_state.py                  # Maintains the robot's state
│   ├── filters
│   │   ├── complementary.py                 # Implements a complementary filter
│   │   └── ekf_filter.py                    # Implements an Extended Kalman Filter
│   ├── motor_control
│   │   └── pid_controller.py                # Implements a PID controller for motor control
│   ├── config
│   │   └── simulation_config.py             # Holds configuration parameters
│   └── utils
│       └── webots_helpers.py                # Utility functions for display and logging
├── package.json                             # Configuration file for npm
├── tsconfig.json                           # Configuration file for TypeScript
└── README.md                                # Documentation for the project
```

## Setup Instructions

1. **Clone the Repository**
   ```bash
   git clone <repository-url>
   cd two_wheel_wall_following
   ```

2. **Install Dependencies**
   Ensure you have the necessary dependencies installed. You can use npm for JavaScript dependencies and pip for Python dependencies.

   ```bash
   npm install
   pip install -r requirements.txt
   ```

3. **Run the Simulation**
   To start the simulation, execute the following command:
   ```bash
   python src/wall_following_controller.py
   ```

## Usage Guidelines

- The robot will automatically follow the inner wall while maintaining a specified distance.
- It will detect obstacles using the LiDAR sensor and adjust its path accordingly to avoid collisions.
- You can modify the wall-following behavior by adjusting parameters in the `simulation_config.py` file.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.