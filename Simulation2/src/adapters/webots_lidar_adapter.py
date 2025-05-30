class WebotsLidarAdapter:
    def __init__(self, lidar, config):
        self.lidar = lidar
        self.config = config
        self.lidar.enable(self.config.LIDAR_TIMESTEP)

    def get_scan(self):
        """Retrieve the latest LiDAR scan data."""
        return self.lidar.getRangeImage()

    def get_point_cloud(self):
        """Retrieve the point cloud data from the LiDAR."""
        distances = self.get_scan()
        angles = np.linspace(-self.config.LIDAR_FOV / 2, self.config.LIDAR_FOV / 2, len(distances))
        points = []

        for distance, angle in zip(distances, angles):
            if distance < self.config.LIDAR_MAX_DISTANCE:
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                points.append((x, y))

        return points

    def get_angles(self):
        """Get the angles corresponding to the LiDAR measurements."""
        return np.linspace(-self.config.LIDAR_FOV / 2, self.config.LIDAR_FOV / 2, self.lidar.getNumberOfPoints())