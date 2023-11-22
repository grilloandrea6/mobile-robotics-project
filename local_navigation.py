import sys
import math

# distance to a waypoint to be considered achieved
EPS = 10

# default constant speed
VEL = 50

# radius of thymio wheels
R = 20

# distance between wheel centers
L = 105

# conversion ratio speed rad/s to thymio's speed 
CONV_RATIO = 65

# 
THRESHOLD = 6000

KP_ALPHA = .5
KP_BETA = .5

#TODO TODO TODO
theta_goal = 0.5

class Local_Navigation(object):
  #def __init__(self, path):
  #  print("init of the class")

  def __init__(self, path):
    self.path = path
    self.waypoint_counter = 0
    self.state = 0 # 0 -> path following / 1 -> obstacle avoidance

  
  def control(self,pose, sensor_data):
    if not self.present_obstacle(sensor_data):
      v,w = self.path_follow(pose)
      return self.differential_steering(v,w)

    else:
      return obstacle_avoidance(sensor_data)


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


  def path_follow(self,pose):
    #pose should be a (x,y,theta) tuple


    objective = self.path[self.waypoint_counter]

    distance = self.dist(pose,objective)

    # check if we reached the objective - non mi piace proprio, voglio verificare se ho superato la retta normale al percorso che volevo fare
    if distance < EPS:
      self.waypoint_counter+=1
      objective = self.path[self.waypoint_counter]
    
    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]
    theta = pose[2]
        
    alpha = (math.atan2(y_diff, x_diff) - theta + math.pi) % (2 * math.pi) - math.pi
    beta = (theta_goal - theta - alpha + math.pi) % (2 * math.pi) - math.pi


    w = KP_ALPHA * alpha + KP_BETA * beta

    v = VEL if alpha > math.pi / 2 or alpha < -math.pi / 2 else -VEL

    return (v,w)
  
  def differential_steering(self,v,w):
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return wl, wr

  def dist(self,a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)     

