def setup_display(robot, width, height):
    """Set up a display for visualizing data in Webots."""
    display = robot.getDevice('display')
    display.setSize(width, height)
    display.setColor(0xFFFFFF)  # Set background color to white
    display.fillRectangle(0, 0, width, height)  # Clear the display
    return display

def log_data(robot_state, slam):
    """Log robot state and SLAM data."""
    # Example logging function
    print(f"Robot State: x={robot_state.x}, y={robot_state.y}, theta={robot_state.theta}")
    print(f"SLAM Map: {slam.get_map()}")