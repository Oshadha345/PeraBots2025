"""
Main entry point for the Webots simulation
This script initializes the simulation environment and runs the controller
"""

import os
import sys
import subprocess

def main():
    """
    Main function to launch the Webots controller
    """
    print("Starting Webots simulation for PeraBots2025...")
    
    # Get the path to the controller
    controller_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'controllers',
        'two_wheel_controller',
        'two_wheel_controller.py'
    )
    
    # Check if the controller file exists
    if not os.path.exists(controller_path):
        print(f"Error: Controller file not found at {controller_path}")
        return
    
    # Get the path to the Webots world file
    # This assumes you have a world file in a worlds directory
    world_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..',
        'worlds',
        'pera_robot.wbt'
    )
    
    # Check if the world file exists
    if not os.path.exists(world_path):
        print(f"Error: World file not found at {world_path}")
        print("Please create a Webots world file for your robot")
        return
    
    # Launch Webots with the specified world
    try:
        # This command will vary depending on your Webots installation
        webots_command = "webots"
        
        # Launch Webots with the world file
        subprocess.run([webots_command, world_path])
        
        print("Webots simulation launched successfully")
    except Exception as e:
        print(f"Error launching Webots: {e}")
        print("Make sure Webots is installed and in your PATH")

if __name__ == "__main__":
    main()