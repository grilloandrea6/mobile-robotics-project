import numpy as np

PROCESS_NOISE_COV =  np.eye(5) * 0.0001  # Adjust the covariance based on your system dynamics
CAMERA_NOISE_COV = np.eye(3) * 0.00001
CAMERA_COVERED_COV = np.eye(3) * 99999999
# SPEED_NOISE_COV = np.array([[373.6623, 103.2730], [103.2730, 189.5869]])
SPEED_NOISE_COV = np.eye(2)*500
MEASUREMENT_NOISE_DEF_COV = np.block([
                            [CAMERA_NOISE_COV, np.zeros((3, 2))],
                            [np.zeros((2, 3)), SPEED_NOISE_COV]
                        ])
MEASUREMENT_NOISE_KIDNAPPING_COV = np.block([
                            [CAMERA_COVERED_COV, np.zeros((3, 2))],
                            [np.zeros((2, 3)), SPEED_NOISE_COV]
                        ])

KIDNAPPED = True
NORMAL = False

class ExtendedKalmanFilter(object):
    def __init__(self):
        print("Init Extended Kalman Filter")
        self.process_noise_cov = PROCESS_NOISE_COV
        self.measurement_noise_cov = MEASUREMENT_NOISE_DEF_COV

    
    def state_initialization(self,initial_state):
        self.state = initial_state
        self.state_dim = len(initial_state)

        # Initialize the error covariance matrix
        self.P = np.eye(self.state_dim)
    
    def set_mode(self, isKidnapped):
        if isKidnapped:
            self.measurement_noise_cov = MEASUREMENT_NOISE_KIDNAPPING_COV
        else:
            self.measurement_noise_cov = MEASUREMENT_NOISE_DEF_COV
    
    def _angle_modulus(self):
        self.state[2] = (self.state[2]-np.pi)%(2*np.pi)-np.pi

    def _state_transition(self, x, u, dt):
        # State transition function for a differential drive robot
        v_bar,omega = u

        x_new = x[0] + v_bar * dt * np.cos(x[2])
        y_new = x[1] + v_bar * dt * np.sin(x[2])
        theta_new = x[2] + omega * dt

        return np.array([x_new, y_new, theta_new, v_bar, omega])

    def _calculate_state_transition_matrix(self, x, u, dt):
        # Jacobian matrix A of the state transition function

        v_bar,omega = u
        #v_right, v_left = u
        #v_bar = (v_right + v_left) / 2.0
        theta_i = x[2]

        A = np.array([
            [1, 0, -v_bar * dt * np.sin(theta_i), 0, 0],
            [0, 1, v_bar * dt * np.cos(theta_i), 0, 0],
            [0, 0, 1, 0, 0],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

        return A

    def _measurement_model(self, x):
        # x, y, and theta are measured usıng the camera
        # v_right and v_left are measured using velocity sensors
        return x

    def _calculate_measurement_jacobian(self, x):
        # Calculate the Jacobian matrix H of the measurement model
        H = np.eye(5)

        return H

    def predict(self, control_input, dt):
        # Update the state transition matrix A based on the control input
        A = self._calculate_state_transition_matrix(self.state, control_input, dt)

        # Predict the state and error covariance
        self.state = self._state_transition(self.state, control_input, dt)
        self.P = A @ self.P @ A.T + self.process_noise_cov

        self._angle_modulus()

    def update(self, measurement):
        # Calculate the measurement Jacobian matrix H
        H = self._calculate_measurement_jacobian(self.state)

        # Calculate the innovation covariance S
        S = H @ self.P @ H.T + self.measurement_noise_cov

        # Calculate the Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update the state and error covariance
        innovation = measurement - self._measurement_model(self.state)
        self.state = self.state + K @ innovation
        self.P = (np.eye(self.state_dim) - K @ H) @ self.P

        self._angle_modulus()