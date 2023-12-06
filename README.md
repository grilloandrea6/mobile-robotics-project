# Basics of Mobile Robotics - Project
Project for the EPFL course Basics of Mobile Robotics. 

2023, fall semester

# Group members
Badil Mujovi (274632)\
Aubin Mercier (316260)\
Mehmet Furkan Dogan (370234)\
Andrea Grillo (371099)

# Demonstration
<center><div><img src = "images\demonstration.gif" width = 350></div></center>

## Introduction
This project aims to combine vision, global and local navigation, and filtering to maneuver a Thymio robot in a predetermined environment.

Our environment consist of a white floor delimited by ArUco markers in the 4 corners. There is an ArUco marker on the Thymio and the goal is an ArUco marker too. Static obstacles are black polygones printed on the map. A camera is placed on top of the environment, to build a map of the static obstacles and compute the optimal path towards the goal.

The Thymio is placed at a random position, the camera tracks its position and orientation, and it has to reach the goal following the optimal path. The system is robust to 3D dynamic obstacles placed along the path, as well as camera covering for short periods of time and kidnapping, that is moving the robot to another place during its activity.

Our implementation includes the following features:
- Computer Vision for detecting obstacles and building the map
- Computer Vision for the tracking of the pose of the Thymio
- Global navigation computes the optimal path from the map using a visibility graph
- Local navigation makes the Thymio follow the computed path as a list of waypoints
- Obstacle avoidance by sensing the environment with proximity sensors
- Extended Kalman Filter allows pose estimation when the camera is covered using the fusion of data from camera and odometry
- Resilience to kidnapping by recomputing the global path when the camera detects that the position of the Thymio has changed
- ...


The main code of the system is implemented in this Jupyter noteboook. The submodules of the system have been implemented in Python classes, each one in a separate file. They are the following:
- Computer Vision - vision_thymio.py
- Global Navigation - global_navigation.py
- Extended Kalman Filter - extended_kalman_filter.py
- Local Navigation - local_navigation.py


Some scripts for testing the subsystems have been written, and they are located under the 'test' directory.
Moreover, for the estimation for the variance matrices for the Kalman Filter, measurements and subsequent data analysis in Matlab have been conducted. All the files related to this part can be found in the 'Sensor Noise Measurement' directory.


Below there is an explanation of each submodule, followed by the explanation of the main loop of the code.
## 1. Computer Vision
TODO

I have found this picture, don't know if it makes sense to use it
<div><img src = "images\detectAruco.png" width = 500 height = 500></div>

## 2. Path Planning
TODO
## 3. Filtering
### Extended Kalman Filter Model
We are using the following model for extended Kalman filter implementation:
$$
x_{i+1} = f(x_i,u_i) + w\\
z_{i+1} = h(x_i) + v
$$

where:

$$
x = 
\begin{bmatrix}
x\\
y\\
\theta\\
\bar{v}\\
\omega
\end{bmatrix}
$$

$$
u = 
\begin{bmatrix}
\bar{v}_{\textrm{sensor}}\\
\omega_{\textrm{sensor}}
\end{bmatrix}
$$

State transition model:
$$
f(x_i,u_i) = 
\begin{bmatrix}
x_{i+1}\\
y_{i+1}\\
\theta_{i+1}\\
\bar{v}_{i+1}\\
\omega_{i+1}
\end{bmatrix} = 
\begin{bmatrix}
x_i + \bar{v}_i \cdot \Delta t \cdot \cos(\theta_i)\\
y_i + \bar{v}_i \cdot \Delta t \cdot \cos(\theta_i)\\
\theta_i + \omega_i \cdot \Delta t\\
\bar{v}_{\textrm{sensor}}\\
\omega_{\textrm{sensor}}
\end{bmatrix}
$$

State transition matrix can be found by calculating the Jacobian of the nonlinear state transition model.

$$
F = 
\begin{bmatrix}
1 & 0 & -\bar{v}\cdot \Delta t \cdot \sin(\theta_i) & 0 & 0\\
0 & 1 & -\bar{v}\cdot \Delta t \cdot \cos(\theta_i) & 0 & 0\\
0 & 0 & 1 & 0 & 0\\
0 & 0 & 0 & 1 & 0\\
0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

State transition can be implemented as:
```py
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
```

Measurement model:
$$
h(x_i) = 
\begin{bmatrix}
x_{\textrm{camera}}\\
y_{\textrm{camera}}\\
\theta_{\textrm{camera}}\\
\bar{v}_{\textrm{sensor}}\\
\omega_{\textrm{sensor}}
\end{bmatrix}
$$

Measurement jacobian:
$$
H = 
\begin{bmatrix}
1 & 0 & 0 & 0 & 0\\
0 & 1 & 0 & 0 & 0\\
0 & 0 & 1 & 0 & 0\\
0 & 0 & 0 & 1 & 0\\
0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

Measurement model can be implemented as:
```py
    def _measurement_model(self, x):
        # x, y, and theta are measured usıng the camera
        # v_right and v_left are measured using velocity sensors
        return x

    def _calculate_measurement_jacobian(self, x):
        # Calculate the Jacobian matrix H of the measurement model
        H = np.eye(5)

        return H
```

### Prediction Step
$$
x_{i+1} = f(x_i,u_i)
$$
$$
P = F \cdot P \cdot F^T + Q
$$
where $P$ is the error covariance matrix and Q is the process noise covariance matrix. We can implement that in pyhon as:

```py
    def predict(self, control_input, dt):
        # Update the state transition matrix A based on the control input
        A = self._calculate_state_transition_matrix(self.state, control_input, dt)

        # Predict the state and error covariance
        self.state = self._state_transition(self.state, control_input, dt)
        self.P = A @ self.P @ A.T + self.process_noise_cov

        self._angle_modulus()
```

### Update Step
Innovation covariance matrix can be calculated as:
$$
S = H \cdot P \cdot H^T + R
$$
Optimal Kalman gain can be calculated as:
$$
K = P \cdot H^T \cdot S^{-1}
$$
Updated state estimate:
$$
x_{i+1} = x_i + K \cdot (z_i - H_i \cdot x_i)
$$
Updated error covariance:
$$
P = (I - K\cdot H)\cdot P
$$

We can implement that in pyhon as:
```py
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
```

### Tuning Covariance Matrices

To tune the measurement covariance matrix, we used the measured speed values when motor is commanded to run at a fixed speeds. You can see the distributions of the wheel speed measurements for different speeds.

<center><div><img src = "images\wheel_speed_distribution.png" width = 350></div></center>

To find the wheel speed sensor noise covariance matrix, the following error distribution is used.

<center><div><img src = "images\measurement_error_distribution.png" width = 350></div></center>

We have found the camera noise covariance matrix in a similar way. Lastly, process noise covariance matrix is tuned by trial and error to get an optimal behaviour.

## 4. Local Navigation
The goal of the Local Navigation submodule is to make the Thymio follow the path computed by the Global Navigation part, while making the Thymio avoid dynamic obstacles that can be placed along the path.

The path is given as a list of tuples (x,y) representing the waypoints of the path. The path is initialized using the `define_path` function. In this function, also the `waypoint_counter` variable is initialized. This variable keeps track of which waypoint we are aiming for at the moment.

#### Control routine
The function which is called at every loop iteration is `control`, whose role is to decide whether to apply the controller to follow the path or the local avoidance routine.
The system starts in the <i>path_follow</i> modality, which is changed to <i>obstacle_avoidance</i> whenever the proximity sensor detect an obstacle. When no obstacle is detected anymore, the obstacle avoidance routine will be run for the following 5 loop cycles before going back to the <i>path_follow</i> mode, to ensure that the obstacle is really been overcome and that the Thymio is far enough from it.


#### Obstacle avoidance routine
The obstacle avoidance routine, as implemented in the `obst_avoid` function, is a simple application of a Neural Network as seen during the course.
<center><div><img src = "images\NNobstacleAvoidance.png" width = 350></div></center>

The input of the network are the readings of the sensors, which are then multiplied by the weights and summed together to get the velocity of the motors.
The two rear sensors of the Thymio have been assigned zero weight, as the Thymio will always move forward so it cannot detect any obstacle from the rear.
To make the Thymio go forward during the obstacle avoidance phase, a constant velocity is added to both neurons as an additional fixed input.

The main difference with the code used during the exercise session, is that in that case the routine has been run on the Thymio using the Aseba transpiler. To integrate the obstacle avoidance in the whole project, the routine is run in Python on the computer. For this reason, the communication latency has to be taken into account, and this leads to the necessity to tune the sampling time in an effective way, as will be explained after.

The weights of the two neurons have been tuned empirically by trial and error. One interesting detail to note is that for the central sensor two different weigths have been chosen for the left and right wheel. This is because otherwise the Thymio could get blocked in case of an obstacle exactly in front of the robot and perpendicular to the longitudinal line of the Thymio. By setting asymmetrical weights, the Thymio will rotate and not get blocked in this situation.



#### Path follow routine
The function `path_follow` has the pose of the Thymio as the only parameter, as the path is defined as an attribute of the Local Navigation class.
The idea is that the two variables to control are the linear and angular velocities, respectively $ v $ and $ \omega $.

The controller chosen is a simple proportional controller for the $ \omega $, and a predefined fixed velocity for $ v $. 

The control law is therefore:
$$ v = v_{fixed} $$
$$ \omega = K_p ( \theta - \gamma) $$
where $ \theta $ is the orientation of the Thymio and $ \gamma $ is the angle of the vector connecting the Thymio to the waypoint. 

### TODO INSERT IMAGE of the angles

The routine checks if the waypoint has been reached, by checking that the actual pose is within a distance $ \epsilon $ of the waypoint. If the condition is true, the `waypoint_counter` variable is incremented and the function is called recursively, until the last goal is reached.

The gain value $ K_p $, the fixed velocity $ v_{fixed} $, and the $ \epsilon $ parameter have been tuned empiricaly by trial and error to obtain the best performance possible of the system as a whole.

### Simulating the controller
To make sure that the controller works before testing it with the Thymio, a simulation has been done as a proof of concept.
The simulation can be done defining a random path and start point, and using the formulas below to integrate the position of the Thymio at every step:
$$ x_{i+1} = x_i + v \cdot cos(\theta) \cdot \Delta t $$
$$ y_{i+1} = y_i + v \cdot sin(\theta) \cdot \Delta t $$
$$ \theta_{i+1} + \theta_i + \omega \cdot \Delta t $$
The result of the simulation can be seen in the graph below:
<center><div><img src = "images\controller.png" width = 500></div></center>

The code used to do the simulation can be found inside the `test` directory, in the `local_navigation_test.py` file.


