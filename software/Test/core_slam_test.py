import sys
import os
import time
import numpy as np
import math
import random
import pygame

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from pera_slam import RPLidarA1, ParticleFilterSLAM
from pera_slam.core import Position

# Constants for pygame visualization
WINDOW_SIZE = 800
MAP_SIZE_PIXELS = 500  # SLAM map pixels
MAP_SIZE_METERS = 10.0
PIXELS_PER_METER = WINDOW_SIZE / MAP_SIZE_METERS # Pygame display scaling

# Navigation parameters
ROBOT_SPEED_MMS = 100  # Speed in mm/s
TURN_SPEED_DEG = 20  # Degrees per navigation step for turns
DT_SECONDS = 0.1     # Time step for simulation updates

OBSTACLE_THRESHOLD_MM = 300  # If obstacle closer than this, turn
WALL_FOLLOW_DISTANCE_MM = 500 # Desired distance from left wall
WALL_ADJUST_ANGLE_DEG = 10    # Angle to adjust when too close/far from wall
OPEN_SPACE_TURN_DEG = 45      # Angle to turn left if no wall detected

# Add these constants for adaptive speed control
MIN_SPEED_MMS = 50      # Minimum speed when near obstacles
MAX_SPEED_MMS = 200     # Maximum speed in open areas
SAFE_DISTANCE_MM = 700  # Distance considered "safe" for higher speeds
DANGER_DISTANCE_MM = 400  # Slow down if obstacles closer than this

class CustomEnvironment:
    """Simulates a custom environment with curved borders and obstacles"""
    
    def __init__(self, size_meters=10.0, outer_radius_factor=0.9, inner_radius_factor=0.45, num_obstacles=8):
        self.size = size_meters
        self.center_x = self.size / 2
        self.center_y = self.size / 2
        self.outer_radius = (self.size / 2) * outer_radius_factor
        self.inner_radius = (self.size / 2) * inner_radius_factor
        self.outer_border = self._generate_curved_border(self.center_x, self.center_y, self.outer_radius, 40, 0.08)
        self.inner_border = self._generate_curved_border(self.center_x, self.center_y, self.inner_radius, 30, 0.12)
        self.obstacles = self._generate_obstacles(num_obstacles)
        # self.path_points = self._generate_path(50) # No longer using predefined path

    def _generate_curved_border(self, center_x, center_y, radius, num_points, variation_factor=0.1):
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            radius_variation = random.uniform(1-variation_factor, 1+variation_factor) * radius
            x = center_x + radius_variation * math.cos(angle)
            y = center_y + radius_variation * math.sin(angle)
            points.append((x, y))
        return points

    def _generate_obstacles(self, num_obstacles):
        obstacles = []
        min_size, max_size = 0.3, 0.6 # meters
        padding = 0.2 # meters, increased padding for obstacle generation
        for _ in range(num_obstacles):
            width = random.uniform(min_size, max_size)
            height = random.uniform(min_size, max_size)
            for _ in range(100): # More attempts to place obstacles
                angle = random.uniform(0, 2 * math.pi)
                # Ensure obstacles are placed within a reasonable area, not too close to center or edge initially
                distance_from_center = random.uniform(self.inner_radius + max(width,height) + padding, 
                                                      self.outer_radius - max(width,height) - padding)
                
                x = self.center_x + distance_from_center * math.cos(angle) - width/2
                y = self.center_y + distance_from_center * math.sin(angle) - height/2
                
                # Check if obstacle is within bounds (optional, as generation logic should handle this)
                if not (self.inner_radius < x < self.outer_radius - width and \
                        self.inner_radius < y < self.outer_radius - height):
                    # continue # Simple check, can be more robust
                    pass


                overlap = False
                for obs_x, obs_y, obs_w, obs_h in obstacles:
                    # Check for overlap with existing obstacles
                    if not (x + width + padding < obs_x or \
                            x - padding > obs_x + obs_w or \
                            y + height + padding < obs_y or \
                            y - padding > obs_y + obs_h):
                        overlap = True
                        break
                if not overlap:
                    obstacles.append((x, y, width, height))
                    break
        return obstacles

    def get_distance_readings(self, robot_x_m, robot_y_m, robot_theta_deg, angles_deg, max_range_m=12.0):
        readings_mm = []
        for angle_deg in angles_deg:
            absolute_angle_rad = math.radians(robot_theta_deg + angle_deg)
            min_distance_m = max_range_m
            
            # Check outer border
            dist_m = self._ray_polygon_intersection(robot_x_m, robot_y_m, absolute_angle_rad, self.outer_border)
            if dist_m is not None: min_distance_m = min(min_distance_m, dist_m)
            
            # Check inner border
            dist_m = self._ray_polygon_intersection(robot_x_m, robot_y_m, absolute_angle_rad, self.inner_border)
            if dist_m is not None: min_distance_m = min(min_distance_m, dist_m)
            
            # Check obstacles
            for ox_m, oy_m, ow_m, oh_m in self.obstacles:
                rect_m = [(ox_m, oy_m), (ox_m + ow_m, oy_m), (ox_m + ow_m, oy_m + oh_m), (ox_m, oy_m + oh_m)]
                dist_m = self._ray_polygon_intersection(robot_x_m, robot_y_m, absolute_angle_rad, rect_m)
                if dist_m is not None: min_distance_m = min(min_distance_m, dist_m)
            
            readings_mm.append(min_distance_m * 1000)
        return readings_mm

    def _ray_polygon_intersection(self, ray_origin_x, ray_origin_y, ray_angle_rad, polygon_points):
        ray_dir_x = math.cos(ray_angle_rad)
        ray_dir_y = math.sin(ray_angle_rad)
        min_intersect_dist = float('inf')

        for i in range(len(polygon_points)):
            p1 = polygon_points[i]
            p2 = polygon_points[(i + 1) % len(polygon_points)]

            # Line segment p1-p2
            edge_dx = p2[0] - p1[0]
            edge_dy = p2[1] - p1[1]

            # Solve for intersection
            # Ray: P = ray_origin + t * ray_dir
            # Edge: P = p1 + u * edge_dir
            # ray_origin_x + t*ray_dir_x = p1_x + u*edge_dx
            # ray_origin_y + t*ray_dir_y = p1_y + u*edge_dy
            
            denominator = ray_dir_x * edge_dy - ray_dir_y * edge_dx
            if abs(denominator) < 1e-6:  # Parallel lines
                continue

            t = ((p1[0] - ray_origin_x) * edge_dy - (p1[1] - ray_origin_y) * edge_dx) / denominator
            u = -((ray_dir_x * (ray_origin_y - p1[1])) - (ray_dir_y * (ray_origin_x - p1[0]))) / denominator
            
            if t >= 0 and 0 <= u <= 1: # Intersection point is on the ray segment and on the edge segment
                min_intersect_dist = min(min_intersect_dist, t)
        
        return min_intersect_dist if min_intersect_dist != float('inf') else None


def draw_visualization(screen, env, robot_position, path_trace_m, slam_map_prob, current_scans_mm=None, scan_angles_deg=None):
    screen.fill((50, 50, 50))  # Dark gray background

    # --- Draw SLAM Map (as background) ---
    if slam_map_prob is not None:
        # Convert probability map (0-1) to grayscale (0-255)
        # Occupied (high prob) = black, Free (low prob) = white, Unknown (0.5) = gray
        slam_map_gray = ((1 - slam_map_prob) * 255).astype(np.uint8)
        
        # Create a Pygame surface from the numpy array
        map_surface = pygame.Surface((slam_map_gray.shape[0], slam_map_gray.shape[1]))
        pygame.surfarray.blit_array(map_surface, np.repeat(slam_map_gray[:, :, np.newaxis], 3, axis=2)) # Make it 3-channel for blit
        
        # Scale SLAM map to fit Pygame window (assuming SLAM map is world-oriented)
        # We need to map SLAM map pixels to Pygame window pixels
        # SLAM map resolution: MAP_SIZE_METERS / MAP_SIZE_PIXELS
        slam_pixel_to_window_scale = PIXELS_PER_METER * (MAP_SIZE_METERS / MAP_SIZE_PIXELS)
        
        scaled_map_surface = pygame.transform.scale(map_surface, 
                                                   (int(slam_map_gray.shape[0] * slam_pixel_to_window_scale),
                                                    int(slam_map_gray.shape[1] * slam_pixel_to_window_scale)))
        screen.blit(scaled_map_surface, (0, 0))


    # --- Draw Environment (borders and obstacles) ---
    # Outer border
    outer_border_px = [(p[0] * PIXELS_PER_METER, p[1] * PIXELS_PER_METER) for p in env.outer_border]
    pygame.draw.polygon(screen, (150, 150, 255), outer_border_px, 2) # Light blue
    # Inner border
    inner_border_px = [(p[0] * PIXELS_PER_METER, p[1] * PIXELS_PER_METER) for p in env.inner_border]
    pygame.draw.polygon(screen, (150, 255, 150), inner_border_px, 2) # Light green
    # Obstacles
    for ox_m, oy_m, ow_m, oh_m in env.obstacles:
        pygame.draw.rect(screen, (255, 100, 100), # Light red
                         pygame.Rect(ox_m * PIXELS_PER_METER, oy_m * PIXELS_PER_METER,
                                     ow_m * PIXELS_PER_METER, oh_m * PIXELS_PER_METER))

    # --- Draw Path Trace ---
    if len(path_trace_m) > 1:
        path_trace_px = [(p[0] * PIXELS_PER_METER, p[1] * PIXELS_PER_METER) for p in path_trace_m]
        pygame.draw.lines(screen, (255, 255, 0), False, path_trace_px, 2) # Yellow

    # --- Draw Robot ---
    robot_x_px = robot_position.x_mm / 1000.0 * PIXELS_PER_METER
    robot_y_px = robot_position.y_mm / 1000.0 * PIXELS_PER_METER
    pygame.draw.circle(screen, (0, 255, 0), (int(robot_x_px), int(robot_y_px)), 6) # Green robot
    # Draw heading indicator
    heading_len_px = 15
    robot_angle_rad = math.radians(robot_position.theta_degrees)
    end_x_px = robot_x_px + heading_len_px * math.cos(robot_angle_rad)
    end_y_px = robot_y_px + heading_len_px * math.sin(robot_angle_rad)
    pygame.draw.line(screen, (255, 0, 0), (int(robot_x_px), int(robot_y_px)), (int(end_x_px), int(end_y_px)), 3) # Red heading

    # Draw current LiDAR scan if available
    if current_scans_mm is not None and scan_angles_deg is not None:
        robot_x_px = robot_position.x_mm / 1000.0 * PIXELS_PER_METER
        robot_y_px = robot_position.y_mm / 1000.0 * PIXELS_PER_METER
        for i, angle_deg in enumerate(scan_angles_deg):
            # Only draw every few points for cleaner visualization
            if i % 5 == 0:  # Display every 5th reading
                scan_dist_m = current_scans_mm[i] / 1000.0
                if scan_dist_m > 0.1 and scan_dist_m < 10.0:  # Reasonable range check
                    angle_rad = math.radians(robot_position.theta_degrees + angle_deg)
                    end_x = robot_x_px + scan_dist_m * math.cos(angle_rad) * PIXELS_PER_METER
                    end_y = robot_y_px + scan_dist_m * math.sin(angle_rad) * PIXELS_PER_METER
                    # Use color based on distance: red for close, yellow for mid, green for far
                    color_intensity = min(255, int(scan_dist_m / 10.0 * 255))
                    scan_color = (255, color_intensity, 0)
                    pygame.draw.line(screen, scan_color, (int(robot_x_px), int(robot_y_px)), 
                                    (int(end_x), int(end_y)), 1)

    pygame.display.flip()


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Autonomous SLAM Explorer")
    clock = pygame.time.Clock()

    env = CustomEnvironment(size_meters=MAP_SIZE_METERS, num_obstacles=random.randint(5,10))
    lidar = RPLidarA1(offset_mm=0) # Assuming RPLidarA1 provides scan_size
    
    slam = ParticleFilterSLAM(laser=lidar, 
                              map_size_pixels=MAP_SIZE_PIXELS, 
                              map_size_meters=MAP_SIZE_METERS, 
                              map_quality=50, # Default from BreezySLAM
                              hole_width_mm=600, # Default
                              num_particles=100) # Default

    # Initial robot position: Start near a corner, e.g., bottom-left, facing towards center
    # Ensure it's within the outer border and not too close to the inner one.
    start_x_m = env.outer_radius * 0.2  # Some distance from the absolute corner
    start_y_m = env.outer_radius * 0.2
    # Try to find a clear starting spot
    for _ in range(10): # Try a few times to find a clear spot
        test_x = random.uniform(env.inner_radius * 1.5, env.outer_radius * 0.5)
        test_y = random.uniform(env.inner_radius * 1.5, env.outer_radius * 0.5)
        # Check if this point is too close to an obstacle
        clear = True
        for ox,oy,ow,oh in env.obstacles:
            if ox < test_x < ox+ow and oy < test_y < oy+oh:
                clear = False
                break
        if clear:
            start_x_m = test_x
            start_y_m = test_y
            break

    start_theta_deg = 45 # Initial angle, e.g., towards the center
    robot_position = Position(start_x_m * 1000, start_y_m * 1000, start_theta_deg)
    
    path_trace_m = [(start_x_m, start_y_m)] # Store path in meters

    running = True
    exploration_steps = 0
    max_exploration_steps = 2000 # Limit exploration time

    while running and exploration_steps < max_exploration_steps:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                # Add a key to visualize just the SLAM map:
                if event.key == pygame.K_m:
                    # Display only the SLAM map
                    slam_map_img = ((1 - slam.get_map()) * 255).astype(np.uint8)
                    pygame.image.save(pygame.surfarray.make_surface(
                        np.repeat(slam_map_img[:, :, np.newaxis], 3, axis=2)), 
                        "slam_map_current.png")
                    print("Saved current SLAM map as slam_map_current.png")

        # --- Get LiDAR Scans ---
        scan_angles_deg = np.linspace(0, 359, lidar.scan_size, endpoint=False) # Full 360° scan
        robot_x_m = robot_position.x_mm / 1000.0
        robot_y_m = robot_position.y_mm / 1000.0
        
        # Get distances in all directions
        current_scans_mm = env.get_distance_readings(robot_x_m, robot_y_m, robot_position.theta_degrees, scan_angles_deg)

        # --- Adaptive Speed Control ---
        # Find closest obstacle in any direction
        min_distance_mm = min(current_scans_mm)

        # Adapt speed based on proximity to obstacles
        if min_distance_mm > SAFE_DISTANCE_MM:
            # Open space - use maximum speed
            robot_speed = MAX_SPEED_MMS
            print(f"Open space detected! Distance: {min_distance_mm:.0f}mm - Speed up to {robot_speed}mm/s")
        elif min_distance_mm < DANGER_DISTANCE_MM:
            # Near obstacle - use minimum speed
            robot_speed = MIN_SPEED_MMS
            print(f"Obstacle nearby! Distance: {min_distance_mm:.0f}mm - Slowing to {robot_speed}mm/s")
        else:
            # Linear interpolation between min and max speeds
            speed_range = MAX_SPEED_MMS - MIN_SPEED_MMS
            distance_range = SAFE_DISTANCE_MM - DANGER_DISTANCE_MM
            speed_factor = (min_distance_mm - DANGER_DISTANCE_MM) / distance_range
            robot_speed = MIN_SPEED_MMS + speed_factor * speed_range
            print(f"Adjusting speed to {robot_speed:.0f}mm/s based on distance {min_distance_mm:.0f}mm")

        # --- Navigation Logic (Enhanced 360° awareness) ---
        # Define sectors for better spatial awareness
        front_dist_mm = float('inf')  # 330° to 30° (front)
        left_dist_mm = float('inf')   # 60° to 120° (left)
        right_dist_mm = float('inf')  # 240° to 300° (right)
        back_dist_mm = float('inf')   # 150° to 210° (back)

        for i, angle_deg in enumerate(scan_angles_deg):
            # Normalize angle to be between -180 and 180 for easier comparison
            norm_angle = (angle_deg + 180) % 360 - 180
            
            if -30 <= norm_angle <= 30:
                front_dist_mm = min(front_dist_mm, current_scans_mm[i])
            elif 60 <= norm_angle <= 120:
                left_dist_mm = min(left_dist_mm, current_scans_mm[i])
            elif -120 <= norm_angle <= -60:
                right_dist_mm = min(right_dist_mm, current_scans_mm[i])
            elif norm_angle >= 150 or norm_angle <= -150:
                back_dist_mm = min(back_dist_mm, current_scans_mm[i])

        dxy_mm = 0
        dtheta_deg = 0

        # Updated navigation logic with 360° awareness
        if front_dist_mm < OBSTACLE_THRESHOLD_MM:
            # Obstacle in front, check right and left to decide turn direction
            if right_dist_mm > left_dist_mm:
                # More space to the right, turn right
                dtheta_deg = -TURN_SPEED_DEG
                print(f"Obstacle Front! Dist: {front_dist_mm:.0f}mm. Turning right (more space).")
            else:
                # More space to the left, turn left
                dtheta_deg = TURN_SPEED_DEG
                print(f"Obstacle Front! Dist: {front_dist_mm:.0f}mm. Turning left (more space).")
        else:
            # No immediate obstacle, proceed with wall following
            dxy_mm = robot_speed * DT_SECONDS  # Apply adaptive speed
            
            if left_dist_mm > WALL_FOLLOW_DISTANCE_MM + 150: # If wall is too far or lost
                dtheta_deg = OPEN_SPACE_TURN_DEG * (DT_SECONDS / 0.1) # Turn left to find wall
                print(f"Left wall far/lost. Dist: {left_dist_mm:.0f}mm. Turning left sharply.")
            elif left_dist_mm < WALL_FOLLOW_DISTANCE_MM - 100: # Too close to left wall
                dtheta_deg = -WALL_ADJUST_ANGLE_DEG * (DT_SECONDS / 0.1) # Turn slightly right
                print(f"Left wall too close. Dist: {left_dist_mm:.0f}mm. Adjusting right.")
            elif left_dist_mm > WALL_FOLLOW_DISTANCE_MM + 100: # Too far from left wall
                dtheta_deg = WALL_ADJUST_ANGLE_DEG * (DT_SECONDS / 0.1) # Turn slightly left
                print(f"Left wall too far. Dist: {left_dist_mm:.0f}mm. Adjusting left.")
            else:
                # Maintaining good distance, continue straight or small adjustments
                print(f"Following left wall. Dist: {left_dist_mm:.0f}mm.")
                pass


        # --- Update Robot Pose (Simulated Odometry) ---
        # Predict new position
        new_x_mm = robot_position.x_mm + dxy_mm * math.cos(math.radians(robot_position.theta_degrees))
        new_y_mm = robot_position.y_mm + dxy_mm * math.sin(math.radians(robot_position.theta_degrees))
        new_x_m = new_x_mm / 1000.0
        new_y_m = new_y_mm / 1000.0

        # Check for collision with borders and obstacles
        inside_outer = point_in_polygon(new_x_m, new_y_m, env.outer_border)
        outside_inner = not point_in_polygon(new_x_m, new_y_m, env.inner_border)
        not_in_obstacle = not collides_with_obstacle(new_x_m, new_y_m, env.obstacles)

        # Track if robot has been stuck
        if exploration_steps % 50 == 0:
            # Check if robot has moved enough in the last 50 steps
            if len(path_trace_m) > 50:
                recent_x = [p[0] for p in path_trace_m[-50:]]
                recent_y = [p[1] for p in path_trace_m[-50:]]
                x_range = max(recent_x) - min(recent_x)
                y_range = max(recent_y) - min(recent_y)
                
                if x_range < 0.2 and y_range < 0.2:  # Robot barely moved
                    # Perform a recovery action - random turn and move
                    print("Robot seems stuck! Performing recovery maneuver")
                    dtheta_deg = random.choice([-90, 90, 180])  # Random turn
                    robot_position.theta_degrees = (robot_position.theta_degrees + dtheta_deg) % 360
                    dxy_mm = 0  # Don't move forward during recovery turn

        if inside_outer and outside_inner and not_in_obstacle:
            # Move is valid
            robot_position.x_mm = new_x_mm
            robot_position.y_mm = new_y_mm
            robot_position.theta_degrees = (robot_position.theta_degrees + dtheta_deg) % 360
        else:
            # Collision detected, turn to avoid
            dxy_mm = 0
            # Choose a random direction to turn to avoid getting stuck
            turn_direction = random.choice([-1, 1])  # -1 = right, 1 = left
            dtheta_deg = turn_direction * (30 + random.random() * 20)  # Random turn between 30-50 degrees
            robot_position.theta_degrees = (robot_position.theta_degrees + dtheta_deg) % 360
            print(f"Collision avoided! Turning {dtheta_deg:.1f} degrees")

        # Define pose_change AFTER all position updates
        pose_change_for_slam = (dxy_mm, dtheta_deg, DT_SECONDS)

        # --- Update SLAM ---
        slam.update(scans_mm=current_scans_mm, 
                    pose_change=pose_change_for_slam, 
                    scan_angles_degrees=scan_angles_deg)
        
        # Get updated robot position from SLAM (optional, but good for consistency if SLAM corrects pose)
        # For this simulation, we'll primarily use our simulated ground truth for robot movement display,
        # but SLAM internally uses its particle filter estimate.
        # slam_est_x_mm, slam_est_y_mm, slam_est_theta_deg = slam.get_position()

        # --- Store Path and Visualize ---
        path_trace_m.append((robot_position.x_mm / 1000.0, robot_position.y_mm / 1000.0))
        
        current_slam_map_prob = slam.get_map() # Get probability map (0 to 1)
        # Update the visualization call to include the LiDAR data:
        draw_visualization(screen, env, robot_position, path_trace_m, slam.get_map(), current_scans_mm, scan_angles_deg)
        
        clock.tick(10) # Control simulation speed (e.g., 10 FPS for visualization)
        exploration_steps += 1
        print(f"Step: {exploration_steps}, Robot @ ({robot_position.x_mm/1000:.1f}, {robot_position.y_mm/1000:.1f})m, {robot_position.theta_degrees:.0f}deg")


    print("Exploration complete or max steps reached.")

    # Save the final SLAM map as an image and numpy array
    final_map = slam.get_map()
    np.save("final_slam_map.npy", final_map)
    try:
        from PIL import Image
        img = Image.fromarray(((1 - final_map) * 255).astype(np.uint8))
        img.save("final_slam_map.png")
        print("Final SLAM map saved as final_slam_map.npy and final_slam_map.png")
    except ImportError:
        print("PIL not installed, only saved as numpy array.")

    # Keep window open until user closes it
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                pygame.quit()
                return
        clock.tick(10)

def point_in_polygon(x, y, polygon):
    """Ray casting algorithm for point-in-polygon test."""
    num = len(polygon)
    j = num - 1
    c = False
    for i in range(num):
        if ((polygon[i][1] > y) != (polygon[j][1] > y)) and \
           (x < (polygon[j][0] - polygon[i][0]) * (y - polygon[i][1]) / (polygon[j][1] - polygon[i][1] + 1e-10) + polygon[i][0]):
            c = not c
        j = i
    return c

def collides_with_obstacle(x, y, obstacles):
    for ox, oy, ow, oh in obstacles:
        if ox <= x <= ox + ow and oy <= y <= oy + oh:
            return True
    return False

# Better collision detection with all sensors
def check_safe_move(new_x_m, new_y_m, env, min_distance_mm):
    # Check environment boundaries
    inside_outer = point_in_polygon(new_x_m, new_y_m, env.outer_border)
    outside_inner = not point_in_polygon(new_x_m, new_y_m, env.inner_border)
    not_in_obstacle = not collides_with_obstacle(new_x_m, new_y_m, env.obstacles)
    
    # Also consider LiDAR readings - if we're too close to something, don't move
    safe_distance = min_distance_mm > DANGER_DISTANCE_MM * 0.7
    
    return inside_outer and outside_inner and not_in_obstacle and safe_distance

if __name__ == "__main__":
    main()
    