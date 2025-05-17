import pygame
import random
import sys
import math

# Initialize pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 800
ARENA_SIZE = 700
OBSTACLE_MIN_SIZE = 30
OBSTACLE_MAX_SIZE = 60
NUM_OBSTACLES = random.randint(6, 10)
BORDER_WIDTH = 15
LINE_WIDTH = 5
OUTER_RADIUS_FACTOR = 0.90  # Percentage of arena size for outer border
INNER_RADIUS_FACTOR = 0.45  # Percentage of arena size for inner border

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)  # For showing intersection points
BORDER_COLOR = (0, 0, 0)  # Black border like in the image

# Setup the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Random Track Generator")

def generate_curved_border(center_x, center_y, radius, num_points, variation_factor=0.1):
    """Generate a curved border"""
    points = []
    
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        # Add random variation to make it irregular/curvy
        radius_variation = random.uniform(1-variation_factor, 1+variation_factor) * radius
        x = center_x + radius_variation * math.cos(angle)
        y = center_y + radius_variation * math.sin(angle)
        points.append((x, y))
    
    return points

def point_in_ring(p, center, inner_radius, outer_radius):
    """Check if point p is within the ring between inner_radius and outer_radius"""
    dist = math.sqrt((p[0] - center[0])**2 + (p[1] - center[1])**2)
    return inner_radius < dist < outer_radius

def generate_obstacles_between_rings(num_obstacles, center, inner_radius, outer_radius, min_size, max_size):
    """Generate obstacles positioned between two circular rings"""
    obstacles = []
    arena_center_x, arena_center_y = center
    
    # Padding to keep obstacles away from the borders
    padding = 15
    
    # Generate obstacles
    for _ in range(num_obstacles):
        # Random size
        width = random.randint(min_size, max_size)
        height = random.randint(min_size, max_size)
        
        # Find a valid position in the ring
        valid_pos = False
        max_attempts = 50
        attempts = 0
        
        while not valid_pos and attempts < max_attempts:
            # Random angle
            angle = random.uniform(0, 2 * math.pi)
            # Random distance between inner and outer radius
            distance = random.uniform(inner_radius + padding, outer_radius - padding - max(width, height)/2)
            
            # Calculate x,y from polar coordinates
            x = arena_center_x + distance * math.cos(angle) - width/2
            y = arena_center_y + distance * math.sin(angle) - height/2
            
            # Create the obstacle rectangle
            obstacle = pygame.Rect(x, y, width, height)
            
            # Check for overlap with existing obstacles
            overlap = False
            for obs in obstacles:
                if obstacle.colliderect(obs.inflate(15, 15)):  # Add padding for spacing
                    overlap = True
                    break
            
            if not overlap:
                valid_pos = True
                obstacles.append(obstacle)
            
            attempts += 1
    
    return obstacles

def generate_random_line(center, outer_radius):
    """Generate a random line that crosses through the arena"""
    center_x, center_y = center
    
    # Generate two random points on the edge of the arena
    angle1 = random.uniform(0, 2 * math.pi)
    angle2 = (angle1 + math.pi + random.uniform(-math.pi/4, math.pi/4)) % (2 * math.pi)
    
    # Calculate points on the edge with some padding beyond the outer border
    radius_with_padding = outer_radius * 1.2
    point1 = (
        center_x + radius_with_padding * math.cos(angle1),
        center_y + radius_with_padding * math.sin(angle1)
    )
    point2 = (
        center_x + radius_with_padding * math.cos(angle2),
        center_y + radius_with_padding * math.sin(angle2)
    )
    
    return point1, point2

def line_intersects_rect(line_start, line_end, rect):
    """Check if a line intersects with a rectangle"""
    # Get rectangle corners
    top_left = (rect.left, rect.top)
    top_right = (rect.right, rect.top)
    bottom_left = (rect.left, rect.bottom)
    bottom_right = (rect.right, rect.bottom)
    
    # Check if line intersects any of the rectangle's sides
    sides = [
        (top_left, top_right),
        (top_right, bottom_right),
        (bottom_right, bottom_left),
        (bottom_left, top_left)
    ]
    
    for side in sides:
        if line_segments_intersect(line_start, line_end, side[0], side[1]):
            return True
    
    # Also check if either endpoint is inside the rectangle
    if rect.collidepoint(line_start) or rect.collidepoint(line_end):
        return True
    
    return False

def line_segments_intersect(p1, p2, p3, p4):
    """Check if line segment p1-p2 intersects with line segment p3-p4"""
    def ccw(a, b, c):
        # Counter-clockwise test
        return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
    
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

def find_polygon_line_intersections(line_start, line_end, polygon_points):
    """Find all intersection points between a line and a polygon"""
    intersections = []
    
    # Go through each edge of the polygon
    for i in range(len(polygon_points)):
        j = (i + 1) % len(polygon_points)
        edge_start = polygon_points[i]
        edge_end = polygon_points[j]
        
        # Check if the line intersects with this edge
        if line_segments_intersect(line_start, line_end, edge_start, edge_end):
            # Calculate the exact intersection point
            intersection = line_intersection(line_start, line_end, edge_start, edge_end)
            if intersection:
                intersections.append(intersection)
    
    return intersections

def line_intersection(p1, p2, p3, p4):
    """Find the intersection point of two line segments if they intersect"""
    # Line 1 as p1 + t*(p2-p1)
    # Line 2 as p3 + s*(p4-p3)
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    
    denominator = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    
    # If lines are parallel or coincident
    if denominator == 0:
        return None
    
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denominator
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denominator
    
    # If intersection is outside the line segments
    if ua < 0 or ua > 1 or ub < 0 or ub > 1:
        return None
    
    # Calculate the intersection point
    x = x1 + ua * (x2 - x1)
    y = y1 + ua * (y2 - y1)
    
    return (x, y)

def main():
    # Center of the arena
    center_x = WIDTH // 2
    center_y = HEIGHT // 2
    
    # Calculate outer and inner radii
    outer_radius = (ARENA_SIZE // 2) * OUTER_RADIUS_FACTOR
    inner_radius = (ARENA_SIZE // 2) * INNER_RADIUS_FACTOR
    
    # Generate the curved border points
    outer_border_points = generate_curved_border(
        center_x, center_y, outer_radius, 40, variation_factor=0.08
    )
    
    inner_border_points = generate_curved_border(
        center_x, center_y, inner_radius, 30, variation_factor=0.12
    )
    
    # Generate obstacles between the inner and outer borders
    obstacles = generate_obstacles_between_rings(
        NUM_OBSTACLES,
        (center_x, center_y),
        inner_radius,
        outer_radius,
        OBSTACLE_MIN_SIZE,
        OBSTACLE_MAX_SIZE
    )
    
    # Generate a random red line
    full_line_start, full_line_end = generate_random_line(
        (center_x, center_y), 
        outer_radius
    )
    
    # Find intersections with borders
    outer_intersections = find_polygon_line_intersections(full_line_start, full_line_end, outer_border_points)
    inner_intersections = find_polygon_line_intersections(full_line_start, full_line_end, inner_border_points)
    
    # Find obstacle intersections
    obstacle_intersections = []
    for obs in obstacles:
        if line_intersects_rect(full_line_start, full_line_end, obs):
            # Create a polygon from the rectangle for intersection detection
            rect_points = [
                (obs.left, obs.top),
                (obs.right, obs.top),
                (obs.right, obs.bottom),
                (obs.left, obs.bottom)
            ]
            intersections = find_polygon_line_intersections(full_line_start, full_line_end, rect_points)
            obstacle_intersections.extend(intersections)
    
    # Define the actual red line endpoints (points A and B)
    # We need exactly one intersection with each border
    line_start = None
    line_end = None
    
    if outer_intersections and inner_intersections:
        # Point A - intersection with outer border
        line_start = outer_intersections[0]  # Take the first intersection with outer border
        
        # Point B - intersection with inner border
        line_end = inner_intersections[0]  # Take the first intersection with inner border
    
    running = True
    show_intersections = True  # Toggle for showing intersection points
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    # Regenerate the arena with 'R' key
                    outer_border_points = generate_curved_border(
                        center_x, center_y, outer_radius, 40, variation_factor=0.08
                    )
                    
                    inner_border_points = generate_curved_border(
                        center_x, center_y, inner_radius, 30, variation_factor=0.12
                    )
                    
                    obstacles = generate_obstacles_between_rings(
                        NUM_OBSTACLES,
                        (center_x, center_y),
                        inner_radius,
                        outer_radius,
                        OBSTACLE_MIN_SIZE,
                        OBSTACLE_MAX_SIZE
                    )
                    
                    full_line_start, full_line_end = generate_random_line(
                        (center_x, center_y), 
                        outer_radius
                    )
                    
                    # Recalculate intersections
                    outer_intersections = find_polygon_line_intersections(full_line_start, full_line_end, outer_border_points)
                    inner_intersections = find_polygon_line_intersections(full_line_start, full_line_end, inner_border_points)
                    
                    obstacle_intersections = []
                    for obs in obstacles:
                        if line_intersects_rect(full_line_start, full_line_end, obs):
                            rect_points = [
                                (obs.left, obs.top),
                                (obs.right, obs.top),
                                (obs.right, obs.bottom),
                                (obs.left, obs.bottom)
                            ]
                            intersections = find_polygon_line_intersections(full_line_start, full_line_end, rect_points)
                            obstacle_intersections.extend(intersections)
                    
                    # Update line endpoints to only show A to B
                    if outer_intersections and inner_intersections:
                        line_start = outer_intersections[0]
                        line_end = inner_intersections[0]
                
                if event.key == pygame.K_i:
                    # Toggle intersection points visibility
                    show_intersections = not show_intersections
                    
                if event.key == pygame.K_ESCAPE:
                    running = False
                    
        # Clear the screen
        screen.fill((30, 30, 30))
        
        # Draw white background for arena
        pygame.draw.circle(screen, WHITE, (center_x, center_y), outer_radius + BORDER_WIDTH)
        
        # Draw outer border
        pygame.draw.polygon(screen, BORDER_COLOR, outer_border_points, BORDER_WIDTH)
        
        # Draw inner border
        pygame.draw.polygon(screen, BORDER_COLOR, inner_border_points, BORDER_WIDTH)
        
        # Draw obstacles
        for obs in obstacles:
            pygame.draw.rect(screen, BLACK, obs)
            
        # Draw red line only from point A to point B (outer to inner border intersection)
        if line_start and line_end:
            pygame.draw.line(screen, RED, line_start, line_end, LINE_WIDTH)
            
            # Draw points A and B with labels
            font = pygame.font.SysFont('Arial', 16)
            
            # Point A (outer border intersection)
            pygame.draw.circle(screen, GREEN, (int(line_start[0]), int(line_start[1])), 6)
            text_a = font.render('A', True, RED)
            text_a_rect = text_a.get_rect(center=(int(line_start[0])+15, int(line_start[1])+15))
            screen.blit(text_a, text_a_rect)
            
            # Point B (inner border intersection)
            pygame.draw.circle(screen, GREEN, (int(line_end[0]), int(line_end[1])), 6)
            text_b = font.render('B', True, RED)
            text_b_rect = text_b.get_rect(center=(int(line_end[0])+15, int(line_end[1])+15))
            screen.blit(text_b, text_b_rect)
        
        # Draw additional intersection points if enabled
        if show_intersections:
            # Skip drawing points A and B again since we already did that
            # Just draw obstacle intersections
            for point in obstacle_intersections:
                pygame.draw.circle(screen, GREEN, (int(point[0]), int(point[1])), 6)
        
        # Update the display
        pygame.display.flip()
        
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()