# simulation/src/pera_slam/visualization.py
import numpy as np
import matplotlib.pyplot as plt

class SLAMMapVisualizer:
    """Class for visualizing SLAM maps with robot position and path"""
    
    def __init__(self, slam_map, robot_position=None, path=None):
        """Initialize visualizer with SLAM map, robot position, and path"""
        self.slam_map = slam_map
        self.robot_position = robot_position
        self.path = path
    
    @staticmethod   
    def visualize_map(slam_map, robot_position=None, path=None, save_path=None):
        """Visualize SLAM map with optional robot position and path"""
        plt.figure(figsize=(10, 10))
        
        # Get map data
        if hasattr(slam_map, 'get_map'):
            map_data = slam_map.get_map()
        else:
            map_data = slam_map
            
        # Display map
        plt.imshow(map_data, cmap='gray', origin='lower')
        
        # Add robot position if provided
        if robot_position is not None:
            if hasattr(robot_position, 'x_mm') and hasattr(robot_position, 'y_mm'):
                # Position object with mm attributes
                x = robot_position.x_mm / 1000.0
                y = robot_position.y_mm / 1000.0
                theta = np.radians(robot_position.theta_degrees)
            else:
                # Position as tuple/list (meters)
                x = robot_position[0]
                y = robot_position[1]
                theta = robot_position[2] if len(robot_position) > 2 else 0
            
            # Convert to map coordinates
            if hasattr(slam_map, 'world_to_map'):
                map_x, map_y = slam_map.world_to_map(x, y)
            else:
                # Estimate conversion if map object doesn't have conversion method
                map_size = map_data.shape[0]
                map_size_meters = 20.0  # Default
                map_x = int((x + map_size_meters/2) / map_size_meters * map_size)
                map_y = int((y + map_size_meters/2) / map_size_meters * map_size)
            
            # Draw robot
            plt.plot(map_x, map_y, 'ro', markersize=10)
            
            # Draw direction
            arrow_length = 20
            end_x = map_x + arrow_length * np.cos(theta)
            end_y = map_y + arrow_length * np.sin(theta)
            plt.arrow(map_x, map_y, end_x - map_x, end_y - map_y, 
                    head_width=5, head_length=10, fc='red', ec='red')
        
        # Add path if provided
        if path is not None and len(path) > 0:
            path_x = []
            path_y = []
            
            for point in path:
                if hasattr(slam_map, 'world_to_map'):
                    map_x, map_y = slam_map.world_to_map(point[0], point[1])
                else:
                    # Estimate conversion
                    map_size = map_data.shape[0]
                    map_size_meters = 20.0  # Default
                    map_x = int((point[0] + map_size_meters/2) / map_size_meters * map_size)
                    map_y = int((point[1] + map_size_meters/2) / map_size_meters * map_size)
                
                path_x.append(map_x)
                path_y.append(map_y)
            
            plt.plot(path_x, path_y, 'b-', linewidth=2)
        
        plt.colorbar(label='Occupancy')
        plt.title('SLAM Map')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        
        if save_path:
            plt.savefig(save_path)
        else:
            plt.show()