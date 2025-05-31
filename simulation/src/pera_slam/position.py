class Position:
    """Simple position class for robot localization"""
    
    def __init__(self, x_mm=0, y_mm=0, theta_degrees=0):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.theta_degrees = theta_degrees
    
    def update(self, x_mm=None, y_mm=None, theta_degrees=None):
        """Update position values"""
        if x_mm is not None:
            self.x_mm = x_mm
        if y_mm is not None:
            self.y_mm = y_mm
        if theta_degrees is not None:
            self.theta_degrees = theta_degrees
    
    def __str__(self):
        return f"Position(x={self.x_mm}mm, y={self.y_mm}mm, θ={self.theta_degrees}°)"