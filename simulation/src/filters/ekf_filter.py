# simulation/src/filters/ekf_filter.py
import numpy as np

class ExtendedKalmanFilter:
    """Extended Kalman Filter for state estimation"""
    
    def __init__(self, dim_x, dim_z):
        """Initialize the EKF"""
        self.dim_x = dim_x
        self.dim_z = dim_z
        
        # State vector
        self.x = np.zeros(dim_x)
        
        # State transition matrix
        self.F = np.eye(dim_x)
        
        # Measurement function jacobian
        self.H = np.zeros((dim_z, dim_x))
        
        # Covariance matrices
        self.P = np.eye(dim_x)  # State covariance
        self.Q = np.eye(dim_x)  # Process noise covariance
        self.R = np.eye(dim_z)  # Measurement noise covariance
        
    def predict(self):
        """Predict state forward using state transition model"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, z, HJacobian, Hx):
        """Update state using measurement z"""
        # Calculate Jacobian at current state
        H = HJacobian(self.x)
        
        # Calculate residual
        hx = Hx(self.x)
        y = z - hx
        
        # Calculate Kalman gain
        PHT = self.P @ H.T
        S = H @ PHT + self.R
        K = PHT @ np.linalg.inv(S)
        
        # Update state and covariance
        self.x = self.x + K @ y
        I = np.eye(self.dim_x)
        self.P = (I - K @ H) @ self.P