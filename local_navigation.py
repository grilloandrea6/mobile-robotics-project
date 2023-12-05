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
KP_ALPHA = .6

# weights for obstacle avoidance using proximity sensors
WEIGHT_LEFT = [ 6,  8.5, -9.5,  -8.5, -6]
WEIGHTRIGHT = [-6, -8.5, -12, 8.5,  6]

# base speed of the obstacle avoidance
VEL_OBST_AVOID = 115

# scale factors for proximity sensors
SENSOR_SCALE = 250

# END CONSTANT DEFINITION

class Local_Navigation(object):
  def __init__(self):
    print("Init Local Navigation")

  def define_path(self,path):
    self.path = path
    self.waypoint_counter = 0
    self.obstacle_counter = 0
    
  def control(self, pose, sensor_data):
    if self.present_obstacle(sensor_data):
      self.obstacle_counter = 5

    if self.obstacle_counter > 0 :
      if not self.present_obstacle(sensor_data):
        self.obstacle_counter -= 1

      wl,wr = self.obst_avoid(sensor_data)
      return self.inv_differential_steering(wl,wr), (wl,wr), False
      
    else :
      v,w = self.path_follow(pose)
      return (v,w), self.differential_steering(v,w), (v,w) == (-1,-1)

  def obst_avoid(self,data):
    wsx = wdx = 0

    for i in range(len(data) - 2):
        # Compute outputs of neurons and set motor powers
        wsx = wsx + data[i] * WEIGHT_LEFT[i] / SENSOR_SCALE
        wdx = wdx + data[i] * WEIGHT_RIGHT[i] / SENSOR_SCALE

    return int(VEL_OBST_AVOID + wsx),int(VEL_OBST_AVOID + wdx)
    
  def path_follow(self, pose):
    #pose is a (x,y,theta) tuple
    thymio_angle = self.radToDeg(pose[2])
    objective = self.path[self.waypoint_counter]
    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]
    distance = self.dist(pose,objective)
    angleFollow = self.radToDeg(math.atan2(y_diff,x_diff))

    #print(f"My pose is {pose[0]} {pose[1]} {thymio_angle}")
    #print(f"My objective is {objective}")
    #print(f"Distance is {distance}")
    #print(f"the angle i have to follow {angleFollow}")

    # check if we reached the objective
    if distance < EPS:
      print(f"reached objective number {self.waypoint_counter}")
      self.waypoint_counter+=1
      if self.waypoint_counter >= len(self.path) :
        print("finished!")
        return (-1,-1)
      else:
        return self.path_follow(pose)
      
    # alpha is the delta between the orientation of the thymio and 
    # the heading we have to follow to reach the objective
    # scaled to be in the interval [-180,+180]
    alpha = (thymio_angle - angleFollow + 180) % 360 - 180
    #print(f"DELTA ANGLE DEG: {alpha}")

    return VEL, KP_ALPHA * alpha 
  
  def differential_steering(self, v, w):
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return int(wl), int(wr)

  def inv_differential_steering(self, wl, wr):
    wl_ = wl * 2 * R / CONV_RATIO
    wr_ = wr * 2 * R / CONV_RATIO
    w = (wr_ - wl_) / (2 * L)
    v = (wl_ + wr_) / 4
    return v, w

  def dist(self,a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    
  def radToDeg(self,angle):
    return angle * 180 / math.pi
  
  def present_obstacle(self,sensor_data):
      return sum(sensor_data) > 0
