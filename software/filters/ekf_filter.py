# ekf_filter.py

from filterpy.kalman import ExtendedKalmanFilter
import numpy as np

class EKFLocalization:
    def __init__(self, dt=0.02):
        self.ekf = ExtendedKalmanFilter(dim_x=3, dim_z=3)  # [x, y, theta], sensor: [x_enc, y_enc, theta_imu]
        self.dt = dt
        self._init_filter()

    def _init_filter(self):
        self.ekf.x = np.zeros((3, 1))  # Initial state: [x, y, theta]

        # State transition covariance (process noise)
        self.ekf.Q = np.diag([0.01, 0.01, 0.01])

        # Measurement covariance (sensor noise)
        self.ekf.R = np.diag([0.1, 0.1, 0.1])

        # Initial uncertainty
        self.ekf.P = np.eye(3) * 1.0

    def predict(self, control):
        """
        Predict step of EKF
        control: [v, w] where v=linear velocity, w=angular velocity
        """
        self.ekf.predict_update(z=None, u=control, HJacobian=None, Hx=None, args=None, hx_args=None, residual=None,
                                fx=self._fx, FJacobian=self._jacobian_F)

    def update(self, z):
        """
        z: np.array([x_enc, y_enc, theta_imu])
        """
        self.ekf.update(z, HJacobian=self._jacobian_H, Hx=self._hx)

    def get_state(self):
        return self.ekf.x.flatten()

    # --- Internal Models ---

    def _fx(self, x, u):
        """
        Nonlinear state transition function.
        x: state vector [x, y, theta]
        u: control input [v, w]
        """
        theta = x[2, 0]
        v, w = u
        dt = self.dt
        dx = np.array([
            [x[0, 0] + v * np.cos(theta) * dt],
            [x[1, 0] + v * np.sin(theta) * dt],
            [x[2, 0] + w * dt]
        ])
        return dx

    def _jacobian_F(self, x, u):
        theta = x[2, 0]
        v, _ = u
        dt = self.dt
        return np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0, 1]
        ])

    def _hx(self, x):
        """
        Measurement function: maps state to measurement space
        """
        return x

    def _jacobian_H(self, x):
        return np.eye(3)
