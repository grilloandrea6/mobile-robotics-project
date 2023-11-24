import sys
import math

# distance to a waypoint to be considered achieved
EPS = 1

# default constant speed
VEL = 30

# radius of thymio wheels
R = 20

# distance between wheel centers
L = 105

# conversion ratio speed rad/s to thymio's speed 
CONV_RATIO = 65

# 
THRESHOLD = 6000  

KP_ALPHA = 10


class Local_Navigation(object):
  #def __init__(self, path):
  #  print("init of the class")

  def __init__(self, path):
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


  def path_follow(self, pose):
    #pose is a (x,y,theta) tuple
    objective = self.path[self.waypoint_counter]
    next_objective = self.path[self.waypoint_counter + 1]

    distance = self.dist(pose,objective)

    x_diff, y_diff = objective[0] - pose[0], objective[1] - pose[1]
    thymio_angle = pose[2]
    obj_angle = math.atan2(next_objective[1] - objective[1], next_objective[0] - next_objective[0])


    print(f"My pose is {pose}")
    print(f"My objective is {objective}")
    print(f"Distance is {distance}")

    # check if we reached the objective
    if distance < EPS:
      print(f"reached objective number {self.waypoint_counter}")
      self.waypoint_counter+=1
      if self.waypoint_counter >= len(self.path) - 1:
        print("finished!")
        return (-1,-1)
      else:
        return self.path_follow(pose)
      
    alpha = (math.atan2(y_diff,x_diff) - thymio_angle + math.pi) % ( 2 * math.pi) - math.pi
    beta = (obj_angle - thymio_angle + math.pi) % ( 2 * math.pi) - math.pi
    w = KP_ALPHA * alpha #*( distance * alpha + 1/(1 + distance) * beta)

    return VEL,w 
  

  def differential_steering(self, v, w):
    wl = CONV_RATIO * (2*v - w*L)/(2*R) # left motor speed
    wr = CONV_RATIO * (2*v + w*L)/(2*R) # right motor speed
    
    return int(wl), int(wr)


  def dist(self,a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)     


