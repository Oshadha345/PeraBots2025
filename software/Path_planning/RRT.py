import cv2
import heapq
import random
import numpy as np
from collections import deque

class RRTStar:
    """RRT* algorithm for path planning"""
    
    def __init__(self, map_array, start, goal, max_iterations=1000, step_size=5, 
             goal_sample_rate=0.1, search_radius=20.0):
        """
        Initialize RRT* planner
        map_array: Binary map (0=free, 255=obstacle)
        start: Start position (x, y) in map coordinates
        goal: Goal position (x, y) in map coordinates
        """
        self.map = map_array
    
        # Convert start and goal to simple tuples with scalar values
        self.start = (float(start[0]), float(start[1])) if hasattr(start, '__len__') else (float(start), 0.0)
        self.goal = (float(goal[0]), float(goal[1])) if hasattr(goal, '__len__') else (float(goal), 0.0)
    
        # Rest of initialization code...
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
    
        # Now use the converted values for checking
        start_x, start_y = int(self.start[0]), int(self.start[1])
        goal_x, goal_y = int(self.goal[0]), int(self.goal[1])
    
        # Check if coordinates are within map bounds
        if 0 <= start_y < self.map.shape[0] and 0 <= start_x < self.map.shape[1]:
            if self.map[start_y, start_x] != 0:
                print("WARNING: Start position is in obstacle space!")
        else:
            print("WARNING: Start position is outside map boundaries!")
        
        if 0 <= goal_y < self.map.shape[0] and 0 <= goal_x < self.map.shape[1]:
            if self.map[goal_y, goal_x] != 0:
                print("WARNING: Goal position is in obstacle space!")
        else:
            print("WARNING: Goal position is outside map boundaries!")
            
        # Initialize node list with start node
        self.nodes = [{'x': self.start[0], 'y': self.start[1], 'parent': None, 'cost': 0}]
        self.goal_node = None
    
        # For visualization
        self.tree_edges = []
        
    def plan(self):
        """Plan a path from start to goal"""
        for i in range(self.max_iterations):
            # Sample a random point with bias toward goal
            if random.random() < self.goal_sample_rate:
                random_point = self.goal
            else:
                random_point = self._sample_free()
                
            # Find nearest node in the tree
            nearest_idx = self._find_nearest(random_point)
            nearest_node = self.nodes[nearest_idx]
            
            # Create new node by steering from nearest toward random
            new_node = self._steer(nearest_node, random_point)
            
            # Check if path is collision-free
            if self._check_collision(nearest_node, new_node):
                # Find nearby nodes
                nearby_indices = self._find_near(new_node)
                
                # Connect new_node to best parent
                self._choose_parent(new_node, nearby_indices)
                
                # Add new node to the tree
                self.nodes.append(new_node)
                new_idx = len(self.nodes) - 1
                
                # Add edge for visualization
                if new_node['parent'] is not None:
                    parent = self.nodes[new_node['parent']]
                    self.tree_edges.append(((parent['x'], parent['y']), (new_node['x'], new_node['y'])))
                
                # Rewire the tree
                self._rewire(new_idx, nearby_indices)
                
                # Check if we reached the goal
                if self._distance(new_node, {'x': self.goal[0], 'y': self.goal[1]}) < self.step_size:
                    if self._check_collision(new_node, {'x': self.goal[0], 'y': self.goal[1], 'parent': None}):
                        # Add goal node
                        goal_node = {'x': self.goal[0], 'y': self.goal[1], 'parent': new_idx,
                                     'cost': new_node['cost'] + self._distance(new_node, {'x': self.goal[0], 'y': self.goal[1]})}
                        self.nodes.append(goal_node)
                        self.goal_node = goal_node
                        
                        # Add final edge
                        self.tree_edges.append(((new_node['x'], new_node['y']), (self.goal[0], self.goal[1])))
                        
                        print(f"RRT* found path in {i+1} iterations")
                        return self._extract_path()
        
        print(f"RRT* failed to find path after {self.max_iterations} iterations")
        return None
    
    def _sample_free(self):
        """Sample a random point in free space"""
        while True:
            x = random.randint(0, self.map.shape[1] - 1)
            y = random.randint(0, self.map.shape[0] - 1)
            if self.map[y, x] == 0:  # Free space
                return (x, y)
    
    def _find_nearest(self, point):
        """Find nearest node to the given point"""
        distances = [self._distance({'x': point[0], 'y': point[1]}, node) for node in self.nodes]
        return np.argmin(distances)
    
    def _steer(self, from_node, to_point):
        """Steer from node toward point with limited step size"""
        dx = to_point[0] - from_node['x']
        dy = to_point[1] - from_node['y']
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist < self.step_size:
            new_x, new_y = to_point
        else:
            theta = np.arctan2(dy, dx)
            new_x = from_node['x'] + self.step_size * np.cos(theta)
            new_y = from_node['y'] + self.step_size * np.sin(theta)
            
        return {'x': new_x, 'y': new_y, 'parent': None, 'cost': 0}
    
    def _check_collision(self, from_node, to_node):
        """Check if path between nodes is collision-free"""
        # Use Bresenham's line algorithm
        x1, y1 = int(from_node['x']), int(from_node['y'])
        x2, y2 = int(to_node['x']), int(to_node['y'])
        
        points = self._bresenham(x1, y1, x2, y2)
        for x, y in points:
            if 0 <= x < self.map.shape[1] and 0 <= y < self.map.shape[0]:
                if self.map[y, x] > 0:  # Obstacle
                    return False
        return True
    
    def _bresenham(self, x1, y1, x2, y2):
        """Bresenham's line algorithm for collision checking"""
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while True:
            points.append((x1, y1))
            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
                
        return points
    
    def _find_near(self, node):
        """Find nodes within search_radius of given node"""
        indices = []
        for i, n in enumerate(self.nodes):
            if self._distance(node, n) < self.search_radius:
                indices.append(i)
        return indices
    
    def _choose_parent(self, node, nearby_indices):
        """Choose best parent for node from nearby nodes"""
        if not nearby_indices:
            return
        
        costs = []
        for i in nearby_indices:
            near_node = self.nodes[i]
            if self._check_collision(near_node, node):
                cost = near_node['cost'] + self._distance(near_node, node)
                costs.append((cost, i))
        
        if costs:
            costs.sort()
            min_cost, min_idx = costs[0]
            node['parent'] = min_idx
            node['cost'] = min_cost
    
    def _rewire(self, new_idx, nearby_indices):
        """Rewire tree to maintain optimality"""
        new_node = self.nodes[new_idx]
        
        for i in nearby_indices:
            if i == new_node['parent']:
                continue
                
            near_node = self.nodes[i]
            if self._check_collision(new_node, near_node):
                cost = new_node['cost'] + self._distance(new_node, near_node)
                if cost < near_node['cost']:
                    # Update edge for visualization
                    if near_node['parent'] is not None:
                        old_parent = self.nodes[near_node['parent']]
                        # Remove old edge
                        if ((old_parent['x'], old_parent['y']), (near_node['x'], near_node['y'])) in self.tree_edges:
                            self.tree_edges.remove(((old_parent['x'], old_parent['y']), (near_node['x'], near_node['y'])))
                    
                    # Add new edge
                    self.tree_edges.append(((new_node['x'], new_node['y']), (near_node['x'], near_node['y'])))
                    
                    # Update parent and cost
                    near_node['parent'] = new_idx
                    near_node['cost'] = cost
    
    def _distance(self, node1, node2):
        """Calculate Euclidean distance between nodes"""
        return np.sqrt((node1['x'] - node2['x'])**2 + (node1['y'] - node2['y'])**2)
    
    def _extract_path(self):
        """Extract path from start to goal"""
        if self.goal_node is None:
            return None
            
        path = []
        node = self.goal_node
        
        while node is not None:
            path.append((node['x'], node['y']))
            if node['parent'] is None:
                break
            node = self.nodes[node['parent']]
            
        return path[::-1]  # Reverse to get path from start to goal
    
    def get_tree_edges(self):
        """Get edges of the RRT* tree for visualization"""
        return self.tree_edges

