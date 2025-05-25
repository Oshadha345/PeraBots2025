import numpy as np
import matplotlib.pyplot as plt


# Try to import roboviz
try:
    from .roboviz import MapVisualizer
    HAS_ROBOVIZ = True
except ImportError:
    HAS_ROBOVIZ = False


def visualize_map(slam, with_particles=False, use_roboviz=False, log_file=None):
    """
    Visualize the current SLAM map using either Matplotlib or PyRoboViz.
    Optionally log robot pose and map updates.
    """
    x_mm, y_mm, theta_degrees = slam.get_position()
    x_m, y_m = x_mm / 1000.0, y_mm / 1000.0


    # Log pose and map update
    if log_file is not None:
        with open(log_file, "a") as f:
            f.write(f"POSE: x={x_mm:.1f}mm y={y_mm:.1f}mm theta={theta_degrees:.1f}deg\n")


    if use_roboviz and HAS_ROBOVIZ:
        # Convert map to uint8 grayscale for roboviz
        map_img = (slam.get_map() * 255).astype(np.uint8)
        visualizer = getattr(visualize_map, "_roboviz", None)
        if visualizer is None:
            visualizer = MapVisualizer(
                slam.map.size_pixels,
                slam.map.size_meters,
                title="SLAM Map",
                show_trajectory=True
            )
            visualize_map._roboviz = visualizer
        visualizer.display(x_m, y_m, theta_degrees, map_img.tobytes())
    else:
        # Fallback to Matplotlib
        plt.figure(figsize=(10, 10))
        plt.imshow(slam.get_map().T, cmap='gray', origin='lower')
        x_pix, y_pix = slam.map.world_to_map(x_mm, y_mm)
        plt.plot(x_pix, y_pix, 'ro', markersize=10)
        arrow_length = 20
        dx = arrow_length * np.cos(np.radians(theta_degrees))
        dy = arrow_length * np.sin(np.radians(theta_degrees))
        plt.arrow(x_pix, y_pix, dx, dy, head_width=5, head_length=10, fc='r', ec='r')
        if with_particles and hasattr(slam, 'particles'):
            particle_x = []
            particle_y = []
            for particle in slam.particles:
                x, y = slam.map.world_to_map(particle.x_mm, particle.y_mm)
                particle_x.append(x)
                particle_y.append(y)
            plt.plot(particle_x, particle_y, 'b.', markersize=1)
        plt.title('SLAM Map')
        plt.xlabel('X (pixels)')
        plt.ylabel('Y (pixels)')
        plt.show()



def save_map(slam, filename):
    """Save the current map to a file"""
    map_data = slam.get_map()
    np.save(filename, map_data)
    
def load_map(slam, filename):
    """Load a map from a file"""
    map_data = np.load(filename)
    slam.map.set_map(map_data)
    