# Basics of Mobile Robotics - Project
Project for the EPFL course Basics of Mobile Robotics. 

2023, fall semester

# Group members
Badil Mujovi (274632) - Currently in Robotics, previously did a Bachelor at HEIG-VD in Microengineering \
Aubin Mercier (316260)\
Mehmet Furkan Dogan (370234)\
Andrea Grillo (371099)

# Demonstration
<center><div><img src = "images\demonstration.gif" width = 350></div></center>

## Introduction
This project aims to combine vision, global and local navigation, and filtering to maneuver a Thymio robot in a predetermined environment.

Our environment consist of a white floor delimited by ArUco markers in the 4 corners. There is an ArUco marker on the Thymio and the goal is an ArUco marker too. Static obstacles are black polygons printed on the map. A camera is placed on top of the environment, to build a map of the static obstacles and compute the optimal path towards the goal.

The Thymio is placed at a random position, the camera tracks its position and orientation, and it has to reach the goal following the optimal path. The system is robust to 3D dynamic obstacles placed along the path, as well as camera covering for short periods of time and kidnapping, that is moving the robot to another place during its activity.

Our implementation includes the following features:
- Computer Vision for detecting obstacles and building the map
- Computer Vision for the tracking of the pose of the Thymio
- Global navigation computes the optimal path from the map using a visibility graph
- Local navigation makes the Thymio follow the computed path as a list of waypoints
- Obstacle avoidance by sensing the environment with proximity sensors
- Extended Kalman Filter allows pose estimation when the camera is covered using the fusion of data from camera and odometry
- Resilience to kidnapping by recomputing the global path when the camera detects that the position of the Thymio has changed


The main code of the system is implemented in this Jupyter noteboook. The submodules of the system have been implemented in Python classes, each one in a separate file. They are the following:
- Computer Vision - vision_thymio.py
- Global Navigation - global_navigation.py
- Extended Kalman Filter - extended_kalman_filter.py
- Local Navigation - local_navigation.py


Some scripts for testing the subsystems have been written, and they are located under the 'test' directory.
Moreover, for the estimation for the variance matrices for the Kalman Filter, measurements and subsequent data analysis in Matlab have been conducted. All the files related to this part can be found in the 'Sensor Noise Measurement' directory.

Below there is an explanation of each submodule, followed by the explanation of the main loop of the code.

## 1. Computer Vision
The computer vision used for this project is composed of many parts, such as the correction of distortion of the camera, the perspective correction of the camera feed, the detection of obstacles and the detection of Aruco markers whose use will be explained later. The obstacle detection will be explained in the [Global navigation](#section_id) section because it was implemented for the global navigation. This part has been almost entirely done using the OpenCV library.

### Distortion correction and camera calibration
Before doing any image processing, the camera had to be calibrated to take into account the possible distortion stemming from the optical components of the camera. This calibration has been done following this [OpenCV guide](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

Two set of values are needed to get an undistorted and corrected picture from the camera : the first being the camera matrix and the second the distortion matrix. These two matrices can be obtained by taking pictures of a ChAruco board at different angles. These matrices are of the following form :
$$
M = 
\left( \begin{matrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{matrix} \right)
$$
where $f_x$ and $f_y$ are the focal lengths and $c_x$ and $c_y$ are the optical centers in x and y axis. We also get the distortion coefficients : 
$$
D = 
\left( \begin{matrix} 
k_1 & k_2 & p_1 & p_2 & k_3
\end{matrix}\right)
$$
where the $k_i$ values are the spherical aberration coefficients and the $p_i$ ones are the tangential aberration coeeficients. The exact meaning of these coefficients can be found in the OpenCV guide. In the code, we can use the method <code>cv2.calibrateCamera()</code>. This method takes images of a ChAruco board and extrapolates the matrices. We can then use the <code>cv2.getOptimalNewCameraMatrix()</code> and <code>cv2.undistort()</code> functions to get the undistorded image. The following method in the <code>Vision_Thymio</code> class give us a corrected frame :
```py
def getFrame(self):
    # camera and distortion matrices
    mtx = np.array([[989.7401734,  0.,           653.72226958],
                    [0.,           998.64802591, 347.61109886],
                    [0.,           0.,           1.          ]])
    dist = np.array([[ 0.12258701, -0.76974406, -0.00790144,  0.00456124,  1.16348775]])
    
    _, frame = self.cap.read()
    
    # undistorted frame
    h,  w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

```

### Aruco markers detection
OpenCV has a module to detect Aruco markers which, in our case, are used to detect the border of our working space as well as the positions and orientations of the Thymio and the goal. This detection is done with the <code>cv2.detectMarkers()</code> method. What OpenCV does is that it applies an adaptive thresholding and finds the contours that could be square shapes. Then it does a perspective transformation on each square shapes to check if there is a binary code corresponding to an ID from the dictionnary used. The method simply returns the IDs found and the corners of each marker. The following method in the <code>Vision_Thymio</code> class give us the markers IDs and corners :
```py
    
def initDictAruco(self,type = cv2.aruco.DICT_4X4_50):
    dict = cv2.aruco.getPredefinedDictionary(type)
    return dict

def detectArucoMarkers(self, img):
    marker_ids = []
    corners = []
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Corners are returned in clockwise order starting with top left
    corners, ids, _ = cv2.aruco.detectMarkers(image=img, dictionary=self.dict)
    
    if len(corners) != 0:
        for i in range(len(ids)):
            marker_ids.append(ids[i][0])

    return marker_ids, corners
```
In our case, the markers with IDs 1 and 2 corresponds respectively to the thymio and the goal. The markers with IDs from 3 to 6 give us the corners of our working ROI. The positions of the goal and the Thymio are given by finding the mean value of their corresponding marker's corners. As for the orientation of the Thymio, it is found by taking the vector given by two corners (which are always returned in a specific order). The methods to find these values are <code>getGoalPosition</code> and <code>getGoalPosition</code>.

### Perspective transformation
For our work, we need to find the position of the thymio, the goal and the obstacles with respect to our marked zone. To do that, we find the Arcuo markers of our ROI and get the position of their inner corners. Then, by using the <code>cv2.getPerspectiveTransform()</code> method, we can map this ROI into a new image with corrected perspective and get a transformation matrix. In our case, we measured the size of our ROI in centimeters, which let us get an image with the same aspect ratio. The following method in the <code>Vision_Thymio</code> class gives us the desired result :
```py
def getPerspectiveAndScaling(self, width_roi=48.4, height_roi=31):
    
    # Need to measure the size of the ROI in real life
    ratio_vh = width_roi/height_roi
    transformMatrix = []
    
    # loops until good transform matrix found 
    while len(transformMatrix) == 0:
        _ , img = self.cap.read()
        ids, corners = self.detectArucoMarkers(img)
        wmax = 0
        hmax = 0
        
        if ids != [] and (3 in ids) and (4 in ids) and (5 in ids) and (6 in ids):
            
            # Take the points of each marker closest to ROI
            mark4 = corners[ids.index(4)][0]
            mark3 = corners[ids.index(3)][0]
            mark5 = corners[ids.index(5)][0]
            mark6 = corners[ids.index(6)][0]
            
            pointA = mark4[2,:]
            pointB = mark3[1,:]
            pointC = mark5[0,:]
            pointD = mark6[3,:]

            # Previous ROI
            inputPoints = np.array([pointA, pointB, pointC, pointD],np.float32)
            
            # Dimensions of new ROI with same aspect ratio
            hmax = img.shape[1]
            wmax = hmax*ratio_vh
            
            # New ROI
            outputPoints = np.array([[0, 0],
                            [0, hmax],
                            [wmax, hmax],
                            [wmax, 0]], np.float32)

            transformMatrix = cv2.getPerspectiveTransform(inputPoints, outputPoints)
            
            sizeROI = np.array([hmax, wmax]).astype(int)
            
            scalingFactor = height_roi/hmax
        else:
            transformMatrix = []
            
            print("Can't find markers 3, 4, 5 and 6 for perspective transformation")
            print("Found ids", ids)

    self.transformMatrix, self.sizeROI, self.scalingFactor = transformMatrix, sizeROI, scalingFactor
```
This method also gives us a scaling value in $cm/px$ to compute the real distances obtained later. We then used the <code>getCorrectedImage()</code> method from the <code>Vision_Thymio</code> class to get the transformed image.

## 2. Path Planning
<a id='section_id'></a>
This part of the project is focused on the global navigation and optimal path finding and, if not mentionned otherwise, the functions used can be found in the global_navigation.py file. The path finding is done in three parts : the vision needed to find the static obstacles, the creation of the visibility graph and the computing of the optimal path. We use the following image as an example instead of a camera feed to showcase the steps for the global navigation :
<center><div><img src = "images\shapes.png" width = 500></div></center>


### Vision for the obstacles
The vision needed to find the obstacles has been included in the global navigation module instead of the vision module to simplify the creation of the functions for the optimal path finding. We decided to use black convex polygons as obstacles to simplify their detection. The first step to get the obstacles is to apply some image processing. First, we apply blurring filters (Gaussian and median) to remove noise and small features. Then we apply an adaptive thresholding which lets us apply morphological operations to the image. Those operations are done with <code>cv2.morphologyEx()</code> and <code>cv2.dilate()</code> methods of OpenCV. All this processing is done by the following function :
```py
def prepareImage(frame, threshold = 100):
    kernel = np.ones((2,2),np.uint8)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   
    
    # Noise reduction 
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = cv2.medianBlur(gray, 3)

    _ ,thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    thresh = cv2.morphologyEx(thresh, cv2.MORPH_GRADIENT, kernel)
    thresh = cv2.dilate(thresh, kernel, iterations = 5)
        
    return thresh
```
After the processing, we get the following result :
<center><div><img src = "images\Processed_Image.png" width = 500></div></center>

We can then find the contours of our shapes and reducing them to only the main vertices of the polygons. This is the main reason why simple shapes have been chosen as obstacles, but the code should still work with more complex shapes. The contours are found using the <code>cv2.findContours()</code> method.
```py
def drawSimplifiedContours(contours, img, scalingFactor):
    listContour = list()
    if len(contours) > 0:
        for cont in contours:
            # We try to find the small contours to delete them
            # by looking at the smallest square area that they can fill
            [_ , (height, width), _] = cv2.minAreaRect(cont)
            area = height*width
            if area < 2500: # equivalent to a square of 50x50 px
                continue
            
            # Approximation on contour into few points and scaling of those points
            contour_approx = approx_contour(cont)
            
            contour_scaled = scalePoints(contour_approx, 11.5/scalingFactor)

            # All the points that have been scaled outside of the ROI are given
            # a big value so that the optimal path finding does not consider them
            for point in contour_scaled:
                print("testing point ", point[0,0], point[0,1])
                y_max = img.shape[1]
                x_max = img.shape[0]

                if point[0, 0] > y_max:
                    print("big >y")
                    point[0,0] = BIG_VALUE
                elif point[0,0] < 0:
                    print("big <y")
                    point[0,0] = -BIG_VALUE
                
                if point[0,1] > x_max:
                    print("big >x")
                    point[0,1] = BIG_VALUE
                elif point[0,1] < 0: 
                    print("big <x")
                    point[0,1] = -BIG_VALUE
                print("testing point ", point[0,0], point[0,1])

            
            listContour.append(contour_scaled[:,0,:])
            cv2.drawContours(img, contour_scaled, -1, (0,0,255), 10)
    else:
        print("No contour found")
    
    return img, listContour
```
The previous function returns the list of all the simplified contours, from function <code>approx_contour()</code>, scaled by a value corresponding to a bit more than the length of the Thymio in pixels from from function <code>scalePoints()</code>. The scaling is done by using as a direction the vector given by the sum of the pair of vectors created by using the main vertice and the two adjacent vertices. This vector is nromalized and scaled by the legnth of the Thymio. This method of scaling does not work with concave shape as they would by scaled inward. Here is the resulting image :
<center><div><img src = "images\Scaled_contours.png" width = 500></div></center>
With, in green, the normal contour and, in red, the scaled one. It is possible to see that the bottom vertice of the hexagon is not correct, this is due to the fact that the contour is not detected at the exact position of the vertice after the dilation of the shape. Still, the result is good enough. It is also important to note that some points can end up outside the ROI after the scaling. In our case, the Thymio could still follow the path with the help of the Kalmann filter, but we decided to remove them to make the code more robust. This was done by making those points a big value such that they are not considered for the optimal path planning.

### Visibility graph
For this part, we decided to use a module called PyVisGraph, which can compute the visibility graph if we give the obstacles as a set of points. This module is used because the creation of the visibility graph is optimized and simple to implement, it can find the shortest path. 

As explained before, we get the simplified and scaled contours of the obstacles in a list using the <code>getVisibilityGraph()</code> method from the <code>Vision_Thymio</code> class, to which we add the start and goal position with the <code>addStartAndGoal()</code>. We then use <code>VisGraph()</code> class and <code>build()</code> method to compute the graph. The method this module uses is by checking for all pairs of points if there's ans obstacle between them, and if not connects them.

### Optimal path
The PyVisGraph module has a <code>shortest_path</code> method in the <code>VisGraph()</code> class. This method uses the Djikstra algortihm to compoute the optimal path given a start and goal position. Here is the result obtained with the axample image :
<center><div><img src = "images\Optimal_path.png" width = 500></div></center>




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


