# simulation/src/path_planning/rrt.py
import numpy as np
import random

class RRTStar:
    """RRT* path planning algorithm"""
    
    def __init__(self, map_array, start, goal, max_iterations=1000, step_size=0.2):
        """Initialize RRT* planner"""
        self.map = map_array
        self.start = start
        self.goal = goal
        self.max_iterations = max_iterations
        self.step_size = step_size
        
        # Node representation: (x, y, parent_index, cost)
        self.nodes = [(*start, -1, 0.0)]
        
        # Map dimensions
        if hasattr(map_array, 'shape'):
            self.map_height, self.map_width = map_array.shape
        else:
            self.map_height = 500  # Default
            self.map_width = 500   # Default
            
        # Map metadata
        self.map_size_meters = 20.0  # Default map size in meters
        self.resolution = self.map_size_meters / self.map_width  # meters per pixel
        
    def plan(self):
        """Run the RRT* algorithm to find a path"""
        goal_idx = -1
        
        for i in range(self.max_iterations):
            # Sample random point
            if random.random() < 0.1:  # Bias towards goal
                point = self.goal
            else:
                point = (
                    random.uniform(-self.map_size_meters/2, self.map_size_meters/2),
                    random.uniform(-self.map_size_meters/2, self.map_size_meters/2)
                )
                
            # Find nearest node
            nearest_idx = self._find_nearest(point)
            nearest_node = self.nodes[nearest_idx]
            
            # Steer towards point
            new_node = self._steer(nearest_node, point)
            
            # Skip if new node is too close to nearest
            if self._distance(new_node, nearest_node) < 0.001:
                continue
                
            # Check if obstacle-free
            if not self._collision_free(nearest_node, new_node):
                continue
                
            # Find neighboring nodes
            neighbor_indices = self._find_neighbors(new_node, 1.0)
            
            # Connect to best parent
            min_cost = nearest_node[3] + self._distance(nearest_node, new_node)
            best_parent_idx = nearest_idx
            
            for idx in neighbor_indices:
                neighbor = self.nodes[idx]
                cost = neighbor[3] + self._distance(neighbor, new_node)
                
                if cost < min_cost and self._collision_free(neighbor, new_node):
                    min_cost = cost
                    best_parent_idx = idx
            
            # Add node to tree
            self.nodes.append((new_node[0], new_node[1], best_parent_idx, min_cost))
            new_idx = len(self.nodes) - 1
            
            # Rewire neighbors
            for idx in neighbor_indices:
                neighbor = self.nodes[idx]
                potential_cost = min_cost + self._distance(new_node, neighbor)
                
                if potential_cost < neighbor[3] and self._collision_free(new_node, neighbor):
                    # Update parent and cost
                    self.nodes[idx] = (neighbor[0], neighbor[1], new_idx, potential_cost)
            
            # Check if we can connect to goal
            if self._distance(new_node, self.goal) < self.step_size and self._collision_free(new_node, self.goal):
                goal_cost = min_cost + self._distance(new_node, self.goal)
                self.nodes.append((*self.goal, new_idx, goal_cost))
                goal_idx = len(self.nodes) - 1
                break
                
            # Print progress
            if i % 100 == 0:
                print(f"RRT* planning: {i}/{self.max_iterations} iterations")
        
        # Extract path if goal was reached
        if goal_idx != -1:
            path = self._extract_path(goal_idx)
            return path
        else:
            print("Failed to find path to goal")
            return []
    
    def _find_nearest(self, point):
        """Find index of nearest node to point"""
        distances = [self._distance(node, point) for node in self.nodes]
        return np.argmin(distances)
        
    def _steer(self, from_node, to_point):
        """Steer from node towards point with limited step size"""
        dx = to_point[0] - from_node[0]
        dy = to_point[1] - from_node[1]
        dist = np.sqrt(dx*dx + dy*dy)
        
        if dist <= self.step_size:
            return to_point
        else:
            theta = np.arctan2(dy, dx)
            return (
                from_node[0] + self.step_size * np.cos(theta),
                from_node[1] + self.step_size * np.sin(theta)
            )
    
    def _distance(self, point1, point2):
        """Calculate Euclidean distance between points"""
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        
    def _find_neighbors(self, point, radius):
        """Find indices of nodes within radius of point"""
        return [idx for idx, node in enumerate(self.nodes) 
                if self._distance(node, point) <= radius]
                
    def _collision_free(self, point1, point2):
        """Check if path between points is collision-free"""
        # Convert world coordinates to map coordinates
        p1_map = self._world_to_map(point1[0], point1[1])
        p2_map = self._world_to_map(point2[0], point2[1])
        
        # Use Bresenham's line algorithm to check for collisions
        points = self._bresenham(p1_map[0], p1_map[1], p2_map[0], p2_map[1])
        
        for x, y in points:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                # Check if pixel is occupied (threshold typically 0.65)
                if self.map[y, x] > 0.65:
                    return False
        return True
        
    def _world_to_map(self, x_world, y_world):
        """Convert world coordinates to map pixel coordinates"""
        x_map = int((x_world + self.map_size_meters/2) / self.map_size_meters * self.map_width)
        y_map = int((y_world + self.map_size_meters/2) / self.map_size_meters * self.map_height)
        
        # Ensure coordinates are within map bounds
        x_map = max(0, min(x_map, self.map_width - 1))
        y_map = max(0, min(y_map, self.map_height - 1))
        
        return x_map, y_map
        
    def _bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return points
        
    def _extract_path(self, goal_idx):
        """Extract path from goal to start by following parent links"""
        path = []
        current_idx = goal_idx
        
        while current_idx != -1:
            node = self.nodes[current_idx]
            path.append((node[0], node[1]))
            current_idx = node[2]
            
        return list(reversed(path))