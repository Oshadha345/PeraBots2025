class ExtendedKalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x  # Dimension of the state vector
        self.dim_z = dim_z  # Dimension of the measurement vector
        
        # State vector
        self.x = np.zeros((dim_x, 1))  # Initial state
        self.P = np.eye(dim_x) * 1000  # Initial state covariance
        
        # Process noise covariance
        self.Q = np.eye(dim_x) * 0.1  # Process noise
        
        # Measurement noise covariance
        self.R = np.eye(dim_z) * 0.1  # Measurement noise
        
        # State transition matrix
        self.F = np.eye(dim_x)  # State transition matrix
        
        # Measurement matrix
        self.H = np.zeros((dim_z, dim_x))  # Measurement matrix

    def predict(self):
        """Predict the next state and update the state covariance."""
        self.x = self.F @ self.x  # State prediction
        self.P = self.F @ self.P @ self.F.T + self.Q  # Covariance prediction

    def update(self, z, HJacobian, Hx):
        """Update the state with a new measurement."""
        # Compute the measurement prediction
        z_pred = Hx(self.x)
        
        # Compute the innovation
        y = z - z_pred
        
        # Compute the Jacobian of the measurement function
        H = HJacobian(self.x)
        
        # Compute the innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Compute the Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update the state estimate
        self.x = self.x + K @ y
        
        # Update the state covariance
        I = np.eye(self.dim_x)
        self.P = (I - K @ H) @ self.P