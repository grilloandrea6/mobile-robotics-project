import numpy as np

import extended_kalman_filter

num_iterations = 500
dt = 0.15



# Define initial state, process noise covariance, and measurement noise covariance
initial_state = np.array([0, 0, 0, 0, 0])  # [x, y, theta, v_right, v_left]
process_noise_cov = np.eye(5) * 0.01  # Adjust the covariance based on your system dynamics

camera_noise_cov = np.eye(3) * 0.1
speed_noise_cov = np.array([[373.6623, 103.2730], [103.2730, 189.5869]])
measurement_noise_cov = np.block([
                                    [camera_noise_cov, np.zeros((3, 2))],
                                    [np.zeros((2, 3)), speed_noise_cov]
                                ])

# Set the trackwidth of the robot
trackwidth = 94  # in mm

# Create an ExtendedKalmanFilter instance
ekf = extended_kalman_filter.ExtendedKalmanFilter(initial_state, process_noise_cov, measurement_noise_cov)

# Main loop
for _ in range(num_iterations):
    # Obtain control input and measurement here
    control_input = np.array([omega_r, omega_l])
    measurement = np.array([x_cam, y_cam, theta_cam, omega_r_sensor, omega_l_sensor])

    # Perform prediction and update steps
    ekf.predict(control_input, dt)
    ekf.update(measurement)

    x, y, theta, v_right, v_left = ekf.state
