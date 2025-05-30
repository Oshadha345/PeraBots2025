import numpy as np
import math
from abc import abstractmethod
from .core import CoreSLAM, Position

class SinglePositionSLAM(CoreSLAM):
    """Base class for SLAM algorithms that maintain a single position estimate"""
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                 map_quality=50, hole_width_mm=600):
        """
        Initialize a single-position SLAM algorithm
        """
        super().__init__(laser, map_size_pixels, map_size_meters, 
                        map_quality, hole_width_mm)
        
        # Initialize position at center of map
        init_coord_mm = 500 * map_size_meters  # center of map
        self.position = Position(init_coord_mm, init_coord_mm, 0)
    
    def update(self, scans_mm, pose_change, scan_angles_degrees=None, should_update_map=True):
        """
        Update SLAM with new sensor data
        scans_mm: LiDAR distance readings
        pose_change: (dxy_mm, dtheta_degrees, dt_seconds) from odometry
        scan_angles_degrees: Optional custom scan angles
        should_update_map: Whether to update the map
        """
        # Unpack pose change
        dxy_mm, dtheta_degrees, dt_seconds = pose_change
        
        # Calculate velocities for scan update
        velocity_factor = (1 / dt_seconds) if dt_seconds > 0 else 0
        dxy_mm_dt = dxy_mm * velocity_factor
        dtheta_degrees_dt = dtheta_degrees * velocity_factor
        velocities = (dxy_mm_dt, dtheta_degrees_dt)
        
        # Update scans
        if hasattr(self, 'scan_for_mapbuild'):
            self.scan_for_mapbuild.update(scans_mm, self.hole_width_mm, 
                                        velocities, scan_angles_degrees)
        else:
            print("[WARNING] scan_for_mapbuild not found")
            
        if hasattr(self, 'scan_for_distance'):
            self.scan_for_distance.update(scans_mm, self.hole_width_mm, 
                                        velocities, scan_angles_degrees)
        else:
            print("[WARNING] scan_for_distance not found")
        
        # Update position and map
        self._update_position_and_map(dxy_mm, dtheta_degrees, should_update_map)
        
    
    def _update_position_and_map(self, dxy_mm, dtheta_degrees, should_update_map):
        """Update position estimate and map"""
        # Start at current position
        start_pos = self.position.copy()
        
        # Add effect of velocities
        start_pos.x_mm += dxy_mm * self._cos_theta()
        start_pos.y_mm += dxy_mm * self._sin_theta()
        start_pos.theta_degrees += dtheta_degrees
        
        # Adjust for laser offset
        start_pos.x_mm += self.laser.offset_mm * self._cos_theta()
        start_pos.y_mm += self.laser.offset_mm * self._sin_theta()
        
        # Get new position using algorithm-specific method
        new_position = self._get_new_position(start_pos)
        
        # Update current position
        self.position = new_position.copy()
        self.position.x_mm -= self.laser.offset_mm * self._cos_theta()
        self.position.y_mm -= self.laser.offset_mm * self._sin_theta()
        
        # Update map if requested
        if should_update_map:
            self.map.update(self.scan_for_mapbuild, new_position, 
                          self.map_quality, self.hole_width_mm)
    
    @abstractmethod
    def _get_new_position(self, start_position):
        """Algorithm-specific method to compute new position"""
        pass
    
    def get_position(self):
        """Get the current position of the robot (x, y, theta)"""
        # Extract scalar values from any potential NumPy arrays
        if hasattr(self.robot_state.x, 'shape'):
            # Handle arrays of any size
            x = float(self.robot_state.x[0]) if len(self.robot_state.x.shape) > 0 else float(self.robot_state.x)
            y = float(self.robot_state.y[0]) if len(self.robot_state.y.shape) > 0 else float(self.robot_state.y)
            theta = float(self.robot_state.theta[0]) if len(self.robot_state.theta.shape) > 0 else float(self.robot_state.theta)
        else:
            # Not a NumPy array
            x = float(self.robot_state.x)
            y = float(self.robot_state.y)
            theta = float(self.robot_state.theta)   
        return (x, y, theta)
    def _cos_theta(self):
        """Helper to compute cosine of current orientation"""
        return math.cos(math.radians(self.position.theta_degrees))
    
    def _sin_theta(self):
        """Helper to compute sine of current orientation"""
        return math.sin(math.radians(self.position.theta_degrees))

class DeterministicSLAM(SinglePositionSLAM):
    """SLAM using only odometry (no scan matching)"""
    
    def _get_new_position(self, start_position):
        """Simply return the odometry-based position"""
        return start_position.copy()

class ParticleFilterSLAM(SinglePositionSLAM):
    """SLAM using particle filter for localization"""
    
    def __init__(self, laser, map_size_pixels, map_size_meters, 
                 map_quality=50, hole_width_mm=600,
                 num_particles=100, sigma_xy_mm=100, sigma_theta_degrees=20):
        """
        Initialize particle filter SLAM
        num_particles: Number of particles to use
        sigma_xy_mm: Position noise standard deviation
        sigma_theta_degrees: Orientation noise standard deviation
        """
        super().__init__(laser, map_size_pixels, map_size_meters, 
                        map_quality, hole_width_mm)
        
        self.num_particles = num_particles
        self.sigma_xy_mm = sigma_xy_mm
        self.sigma_theta_degrees = sigma_theta_degrees
        
        # Initialize particles around the starting position
        self.particles = []
        for _ in range(num_particles):
            self.particles.append(Position(
                self.position.x_mm + np.random.normal(0, sigma_xy_mm),
                self.position.y_mm + np.random.normal(0, sigma_xy_mm),
                self.position.theta_degrees + np.random.normal(0, sigma_theta_degrees)
            ))
        
        self.particle_weights = np.ones(num_particles) / num_particles
    
    def _get_new_position(self, start_position):
        """Use particle filter to estimate position"""
        # Move particles according to odometry
        for particle in self.particles:
            # Add random noise to motion model
            particle.x_mm = start_position.x_mm + np.random.normal(0, self.sigma_xy_mm)
            particle.y_mm = start_position.y_mm + np.random.normal(0, self.sigma_xy_mm)
            particle.theta_degrees = start_position.theta_degrees + np.random.normal(0, self.sigma_theta_degrees)
        
        # Compute weights based on scan matching
        for i, particle in enumerate(self.particles):
            # Calculate scan match score
            match_score = self._compute_scan_match_score(particle)
            self.particle_weights[i] = match_score
        
        # Normalize weights
        weight_sum = np.sum(self.particle_weights)
        if weight_sum > 0:
            self.particle_weights /= weight_sum
        else:
            self.particle_weights[:] = 1.0 / self.num_particles
        
        # Resample particles
        indices = np.random.choice(
            self.num_particles, 
            self.num_particles, 
            p=self.particle_weights
        )
        self.particles = [self.particles[i].copy() for i in indices]
        
        # Compute mean position
        mean_x = 0
        mean_y = 0
        mean_cos_theta = 0
        mean_sin_theta = 0
        
        for particle in self.particles:
            mean_x += particle.x_mm
            mean_y += particle.y_mm
            mean_cos_theta += math.cos(math.radians(particle.theta_degrees))
            mean_sin_theta += math.sin(math.radians(particle.theta_degrees))
        
        mean_x /= self.num_particles
        mean_y /= self.num_particles
        mean_cos_theta /= self.num_particles
        mean_sin_theta /= self.num_particles
        
        mean_theta = math.degrees(math.atan2(mean_sin_theta, mean_cos_theta))
        
        return Position(mean_x, mean_y, mean_theta)
    
    def _compute_scan_match_score(self, position):
        """Compute how well a scan matches the map at a given position"""
        # Simple scoring based on ray casting
        score = 1.0
        map_data = self.map.get_map()
        
        # Sample a few rays for efficiency
        sample_indices = np.linspace(0, len(self.scan_for_distance.distances_mm)-1, 10).astype(int)
        
        for idx in sample_indices:
            distance_mm = self.scan_for_distance.distances_mm[idx]
            if distance_mm <= 0:
                continue
                
            angle_rad = math.radians(position.theta_degrees + self.scan_for_distance.angles_deg[idx])
            endpoint_x = position.x_mm + distance_mm * math.cos(angle_rad)
            endpoint_y = position.y_mm + distance_mm * math.sin(angle_rad)
            
            # Convert to map coordinates
            x_pix, y_pix = self.map.world_to_map(endpoint_x, endpoint_y)
            
            if 0 <= x_pix < self.map.size_pixels and 0 <= y_pix < self.map.size_pixels:
                # Higher score if endpoint matches occupancy in map
                if map_data[x_pix, y_pix] > 0.5:  # occupied in map
                    score *= 1.2
            
        return score