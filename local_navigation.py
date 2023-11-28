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

# END CONSTANT DEFINITION

class Local_Navigation(object):
  def __init__(self):
    print("Init Local Navigation")
    self.waypoint_counter = 0
    #self.state = 0 # 0 -> path following / 1 -> obstacle avoidance

  def define_path(self,path):
    self.path = path
    self.waypoint_counter = 0
    
  def control(self, pose, sensor_data):
    if not self.present_obstacle(sensor_data):
      v,w = self.path_follow(pose)
      return self.differential_steering(v,w)
    else:
      return self.obstacle_avoidance(sensor_data)

  def present_obstacle(self,sensor_data):
      # we want to give more weight to the front sensor (2) as we do not want       
      for i in range(len(sensor_data)):
        sum += sensor_data[i]
      
      if sum > 0:
        return True
      else:
        return False


  def obst_avoid(self, data):
    w_l = [6,  11, -10, -11, -6,  0,0]
    w_r = [-6, -11, -12.5,  11,  6, 0,0]

    # Scale factors for sensors
    sensor_scale = 230

    y = [0,0]
    x = [0,0,0,0,0,0,0]

    for i in range(len(x)):
        # Get and scale inputs
        x[i] = data[i] / sensor_scale
        
        # Compute outputs of neurons and set motor powers
        y[0] = y[0] + x[i] * w_l[i]
        y[1] = y[1] + x[i] * w_r[i]

    return int(115 + y[0]),int(115 + y[1])

  def radToDeg(self,angle):
    return angle * 180 / math.pi

  def path_follow(self, pose):
    # ASTOLFI CONTROLLER
    #pose is a (x,y,theta) tuple
    objective = self.path[self.waypoint_counter]
    distance = self.dist(pose,objective)

    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]
    thymio_angle = pose[2]

    print(f"My pose is {pose[0]} {pose[1]} {self.radToDeg(pose[2])}")
    print(f"My objective is {objective}")
    print(f"Distance is {distance}")
    print(f"the angle i have to follow {self.radToDeg(math.atan2(y_diff,x_diff))}")

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
    diff_angle = self.radToDeg(math.atan2(y_diff,x_diff))
    thymio_angle = self.radToDeg(thymio_angle)
    alpha = (thymio_angle - diff_angle + 180) % 360 - 180
    print(f"DELTA ANGLE DEG: {alpha}")

    beta = (thymio_angle + alpha + math.pi) % ( 2 * math.pi) - math.pi

    w = KP_ALPHA * alpha + KP_BETA * beta
    v = 2000 #KP_DIST * distance
    return v,w 
  
  def differential_steering(self, v, w):
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return int(wl), int(wr)

  def dist(self,a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)