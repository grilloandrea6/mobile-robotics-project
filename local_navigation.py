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
CONV_RATIO = 1

# 
THRESHOLD = 6000  

KP_ALPHA = 1
KP_BETA = -.5
KP_DIST = 3

# END CONSTANT DEFINITION

class Local_Navigation(object):
  def __init__(self, path):
    print("init local nav")
    self.path = path
    self.waypoint_counter = 0
    self.state = 0 # 0 -> path following / 1 -> obstacle avoidance

  def control(self, pose, sensor_data):
    if not self.present_obstacle(sensor_data):
      return self.path_follow(pose)
    else:
      return self.obstacle_avoidance(sensor_data)

  def present_obstacle(self,sensor_data):
    if self.state == 0 :
      # we have been following the path until now, we have to check if there's an obstacle
      
      # we want to give more weight to the front sensor (2) as we do not want       
      weighted_sum = sensor_data[0] * 1 + sensor_data[1] * 3 + sensor_data[2] * 10 +  sensor_data[3] * 3 + sensor_data[4] * 1
      if weighted_sum > THRESHOLD:
        self.state = 1
        return True
      else:
        return False
    else:
      # we have been avoiding obstacles, we have to check if we can go back to path following
      #TODO
      return False

  def obstacle_avoidance(self, sensor_data, old_vel):
    w_l = [40,  20, -20, -20, -40,  30, -10, 8, 0]
    w_r = [-40, -20, -20,  20,  40, -10,  30, 0, 8]

    # Scale factors for sensors and constant factor
    sensor_scale = 200
    
    wl = wr = 0
    x = [0,0,0,0,0,0,0,old_vel[0], old_vel[1]]
        
    for i in range(len(x)):
        # Get and scale inputs
        x[i] = sensor_data[i] // sensor_scale
        
        # Compute outputs of neurons and set motor powers
        wl = wl + x[i] * w_l[i]
        wr = wr + x[i] * w_r[i]
    
    # Set motor powers
    return int(wl), int(wr)

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
      
    alpha = (math.atan2(y_diff,x_diff) - thymio_angle + math.pi) % ( 2 * math.pi) - math.pi
    beta = (-thymio_angle -alpha + math.pi) % ( 2 * math.pi) - math.pi

    w = KP_ALPHA * alpha + KP_BETA * beta
    v = KP_DIST * distance
    return v,w 
  
  def differential_steering(self, v, w):
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return int(wl), int(wr)

  def dist(self,a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)