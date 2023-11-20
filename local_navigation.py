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

class Local_Navigation(object):
  def __init__(self, path):
    self.path = path
    self.waypoint_counter = 0
    self.state = 0 # 0 -> path following / 1 -> obstacle avoidance
  
  def dist(x,y):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)     

  def control(pose, sensor_data):
    if not self.present_obstacle(sensor_data):
      v,w = path_follow(pose)
      return differential_steering(v,w)

    else return obstacle_avoidance(sensor_data)


  def present_obstacle(sensor_data):
    if(self.state == 0) 
      # we have been following the path until now, we have to check if there's an obstacle
      
      # we want to give more weight to the front sensor (2) as we do not want       
      weighted_sum = sensor_data[0] * 1 + sensor_data[1] * 3 + sensor_data[2] * 10 +  sensor_data[3] * 3 + sensor_data[4] * 1
      if weighted_sum > THRESHOLD:
        self.state = 1
        return True
      else:
        return False

    else
      





  def path_follow(pose)
    #pose should be a (x,y,theta) tuple

    objective = self.path[self.waypoint_counter]

    distance = dist(pose,objective)

    # check if we reached the objective
    if distance < eps:
      self.waypoint_counter ++
      objective = self.path[self.waypoint_counter]
      distance = dist(pose,objective)

    
    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]

        
    alpha = (math.atan2(y_diff, x_diff) - theta + math.pi) % (2 * math.pi) - math.pi
    beta = (theta_goal - theta - alpha + math.pi) % (2 * math.pi) - math.pi


    w = Kp_alpha * alpha + Kp_beta * beta

    v = VEL if alpha > math.pi / 2 or alpha < -math.pi / 2 else -VEL

    return self.diff_eq(v,w)
  
  def differential_steering(v,w)
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return wl, wr
