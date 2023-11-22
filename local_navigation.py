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
  
  def dist(self,x,y):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)     

  def control(self, pose, sensor_data):
    if not self.present_obstacle(sensor_data):
      v,w = path_follow(pose)
      return differential_steering(v,w)

    else return obstacle_avoidance(sensor_data)


  def present_obstacle(self, sensor_data):
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
      

  def obstacle_avoidance(self, sensor_data, old_vel):
    global prox_horizontal, motor_left_target, motor_right_target, button_center, state

    w_l = [40,  20, -20, -20, -40,  30, -10, 8, 0]
    w_r = [-40, -20, -20,  20,  40, -10,  30, 0, 8]

    # Scale factors for sensors and constant factor
    sensor_scale = 200
    constant_scale = 20
    
    wl = wr = 0
    x = [0,0,0,0,0,0,0,old_vel[0], old_vel[1]]
        
    for i in range(len(x)):
        # Get and scale inputs
        x[i] = prox_horizontal[i] // sensor_scale
        
        # Compute outputs of neurons and set motor powers
        wl = wl + x[i] * w_l[i]
        wr = wr + x[i] * w_r[i]
    
    # Set motor powers
    return int(wl), int(wr)


  def path_follow(self, pose)
    #pose should be a (x,y,theta) tuple

    objective = self.path[self.waypoint_counter]

    distance = dist(pose,objective)

    # check if we reached the objective
    if distance < EPS:
      self.waypoint_counter ++
      objective = self.path[self.waypoint_counter]
      distance = dist(pose,objective)

    
    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]
    theta_diff = math.atan2(y_diff, x_diff)
   
    thymio_angle = pose[2]

    alpha = -actual_angle + theta # maybe not exactly theta
    deta = -actual_angle - alpha


    alpha = ( - theta + math.pi) % (2 * math.pi) - math.pi
    beta = (theta_goal - theta - alpha + math.pi) % (2 * math.pi) - math.pi


    w = Kp_alpha * alpha + Kp_beta * beta

    v = VEL if alpha > math.pi / 2 or alpha < -math.pi / 2 else -VEL

    return self.diff_eq(v,w)
  
  def differential_steering(self, v,w)
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return int(wl), int(wr)
