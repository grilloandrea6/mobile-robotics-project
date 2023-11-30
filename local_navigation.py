import math

# CONSTANT DEFINITION

# distance to a waypoint to be considered achieved
EPS = 5

# default constant speed
VEL = 30

# radius of thymio wheels
R = 20

# distance between wheel centers
L = 105

# conversion ratio speed rad/s to thymio's speed 
CONV_RATIO = 2

# 
THRESHOLD = 6000  

KP_ALPHA = 1
KP_BETA = -.5
KP_DIST = 70

PATH_FOLLOWING = 0
OBSTACLE_AVOIDANCE = 1

ANGLE_THRESH = 5

# END CONSTANT DEFINITION

class Local_Navigation(object):
  def __init__(self):
    print("Init Local Navigation")
    self.waypoint_counter = 0
    self.state = PATH_FOLLOWING # 0 -> path following / 1 -> obstacle avoidance

  def define_path(self,path):
    self.path = path
    self.waypoint_counter = 0
    
  def control(self, pose, sensor_data):
    if self.state == PATH_FOLLOWING:
      if self.present_obstacle(sensor_data):
        self.state == OBSTACLE_AVOIDANCE
        return self.obst_avoid(sensor_data)

      v,w = self.path_follow(pose)
      return self.differential_steering(v,w)
    else:
      _, _, angleFollow = self.getObjective(pose)
      thymioAngle = self.radToDeg(pose[2])
      if abs(angleFollow - thymioAngle) < ANGLE_THRESH and not self.present_obstacle(sensor_data):
        self.state = PATH_FOLLOWING
        v,w = self.path_follow(pose)
        return self.differential_steering(v,w)
      
      return self.obstacle_avoidance(sensor_data)

  def present_obstacle(self,sensor_data):
      if sum(sensor_data) > 0:
        return True
      else:
        return False


  def obst_avoid(self,data):
    w_l = [ 6,  9, -10,  -9, -6]
    w_r = [-6, -9, -12.5, 9,  6]

    # Scale factors for sensors
    sensor_scale = 230

    wsx = wdx = 0

    for i in range(len(data) - 2):
        # Compute outputs of neurons and set motor powers
        wsx = wsx + data[i] * w_l[i] / sensor_scale
        wdx = wdx + data[i] * w_r[i] / sensor_scale

    return int(115 + wsx),int(115 + wdx)

  def radToDeg(self,angle):
    return angle * 180 / math.pi

  def getObjective(self,pose):
    objective = self.path[self.waypoint_counter]
    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]
    distance = self.dist(pose,objective)
    angleFollow = self.radToDeg(math.atan2(y_diff,x_diff))
    return objective, distance, angleFollow



  def path_follow(self, pose):
    # ASTOLFI CONTROLLER
    #pose is a (x,y,theta) tuple
    thymio_angle = self.radToDeg(pose[2])

    objective, distance, angleFollow = self.getObjective(pose)

    print(f"My pose is {pose[0]} {pose[1]} {self.radToDeg(pose[2])}")
    print(f"My objective is {objective}")
    print(f"Distance is {distance}")
    print(f"the angle i have to follow {angleFollow}")

    # check if we reached the objective
    if distance < EPS:
      print(f"reached objective number {self.waypoint_counter}")
      self.waypoint_counter+=1
      if self.waypoint_counter >= len(self.path) :
        print("finished!")
        return (-1,-1)
      else:
        return self.path_follow(pose)
      
    #alpha = (- math.atan2(y_diff,x_diff) + thymio_angle + 5*math.pi) % ( 2 * math.pi) - math.pi
    
    alpha = (thymio_angle - angleFollow + 180) % 360 - 180
    print(f"AAAA - DELTA ANGLE DEG: {alpha}")

    beta = 0 #(thymio_angle + alpha + 180) % 360 - 180

    w = KP_ALPHA * alpha + KP_BETA * beta
    v = 2000 #KP_DIST * distance
    return v,w 
  
  def differential_steering(self, v, w):
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return int(wl), int(wr)

  def dist(self,a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)