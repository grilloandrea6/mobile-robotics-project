# Mobile Robotics Project
# Local Navigation Submodule
# Author: Andrea Grillo
# 2023, fall semester

import math

# CONSTANT DEFINITION

# distance to a waypoint to be considered reached
EPS = 5

# default constant speed
VEL = 2000

# radius of thymio wheels
R = 20

# distance between wheel centers
L = 105

# conversion ratio for wheel speed
CONV_RATIO = 2

# controller proportional constant for angular velocity
KP_ALPHA = .55

# weights for obstacle avoidance using proximity sensors
WEIGHT_LEFT = [ 6,  9, -10,  -9, -6]
WEIGHT_RIGHT = [-6, -8.5, -12.5, 9,  6]

# base speed of the obstacle avoidance
VEL_OBST_AVOID = 115

# scale factors for proximity sensors
SENSOR_SCALE = 250

# counter for the obstacle_avoidance routine
OBSTACLE_COUNTER = 7

# END CONSTANT DEFINITION

class Local_Navigation(object):
  # Class constructor
  def __init__(self):
    print("Init Local Navigation")

  # Initialize the path as a local attribute of the class
  # Initialize the counters
  def define_path(self,path):
    self.path = path
    self.waypoint_counter = 0
    self.obstacle_counter = 0
    
  # Main control routine, chooses between 
  # the obst_avoid and path_follow routines
  def control(self, pose, sensor_data):
    # If there is an obstacle, set the counter
    if self.present_obstacle(sensor_data):
      self.obstacle_counter = OBSTACLE_COUNTER

    # If the counter is > 0
    if self.obstacle_counter > 0 :
      # Decrement the counter if we do not have an obstacle
      if not self.present_obstacle(sensor_data):
        self.obstacle_counter -= 1

      # Run the obst_avoid routine
      wl,wr = self.obst_avoid(sensor_data)
      return self.inv_differential_steering(wl,wr), (wl,wr), False
      
    else :
      # Run the path_follow routine
      v,w = self.path_follow(pose)
      return (v,w), self.differential_steering(v,w), (v,w) == (-1,-1)

  # Obstacle avoidance routine
  def obst_avoid(self,data):
    wsx = wdx = 0

    for i in range(len(data) - 2):
        # Compute outputs of neurons and set motor powers
        wsx = wsx + data[i] * WEIGHT_LEFT[i] / SENSOR_SCALE
        wdx = wdx + data[i] * WEIGHT_RIGHT[i] / SENSOR_SCALE

    return int(VEL_OBST_AVOID + wsx),int(VEL_OBST_AVOID + wdx)
  
  # Path follow routine
  def path_follow(self, pose):
    #pose is a (x,y,theta) tuple
    thymio_angle = self.radToDeg(pose[2])
    objective = self.path[self.waypoint_counter]

    # Difference vector between objective and the Thymio
    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]
    distance = self.dist(pose,objective)

    # The angle of the difference Vector
    angleFollow = self.radToDeg(math.atan2(y_diff,x_diff))

    # Debugging prints
    #print(f"My pose is {pose[0]} {pose[1]} {thymio_angle}")
    #print(f"My objective is {objective}")
    #print(f"Distance is {distance}")
    #print(f"the angle i have to follow {angleFollow}")

    # Check if we reached the next waypoint
    if distance < EPS:
      print(f"reached objective number {self.waypoint_counter}")
      self.waypoint_counter+=1
      if self.waypoint_counter >= len(self.path) :
        print("Finished!")
        return (-1,-1)
      else:
        return self.path_follow(pose)
      
    # alpha is the delta between the orientation of the thymio and 
    # the heading we have to follow to reach the objective
    # scaled to be in the interval [-180,+180]
    alpha = (thymio_angle - angleFollow + 180) % 360 - 180
    #print(f"DELTA ANGLE DEG: {alpha}")

    # We return the linear and angular velocities
    # the linear velocity is a constant
    # the angular velocity is computed by a proportional controller
    return VEL, KP_ALPHA * alpha 
  
  # Differential steering equations
  # Takes linear and angular velocities as input
  # Outputs the velocities of the two wheels
  def differential_steering(self, v, w):
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return int(wl), int(wr)


  # Inverse differential steering equations
  # Takes the velocities of the two wheels as input
  # Outputs the linear and angular velocities
  def inv_differential_steering(self, wl, wr):
    wl_ = wl * 2 * R / CONV_RATIO
    wr_ = wr * 2 * R / CONV_RATIO
    w = (wr_ - wl_) / (2 * L)
    v = (wl_ + wr_) / 4
    return v, w

  # Helper function: distance between 2 points
  def dist(self,a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
  
  # Helper function: radiants to degree conversion
  def radToDeg(self,angle):
    return angle * 180 / math.pi
  
  # Helper function: checks if an obstacle is detected
  def present_obstacle(self,sensor_data):
      return sum(sensor_data) > 0
