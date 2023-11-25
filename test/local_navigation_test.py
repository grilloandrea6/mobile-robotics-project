import sys
import math
sys.path.append('../')
from local_navigation import Local_Navigation
import matplotlib.pyplot as plt 

nodes = [(0,0),(1,0),(3,10),(5,5),(10,2),(12,7)]


local_nav = Local_Navigation(nodes)

#pose = []
#pose.append((0,0,45* math.pi / 180))
x = [0]
y = [0]
theta = 45 * math.pi / 180

for i in range(20000):
    print(f"iterazione numero {i}")
    v,w = local_nav.path_follow((x[-1],y[-1],theta))

    wl,wr= local_nav.differential_steering(v,w)
    print(f"differential steering {(wl,wr)}") 
    if abs(wl) > 4000 or abs(wr) > 4000:
        print(f"SATURATED")
        #input()
    if (v,w) == (-1,-1):
        print("FINISHED")
        break
    #pose.append(   (pose[-1][0] + v * math.cos(pose[-1][2])/400, pose[-1][1] + v * math.sin(pose[-1][2])/400, pose[-1][2] + w/400) )
    x.append(x[-1] + v *  math.cos(theta)/400)
    y.append( y[-1] + v * math.sin(theta)/400)
    theta = theta + w/400
    

    plt.arrow(x[-1],y[-1],v*math.cos(theta)/600,v*math.sin(theta)/600)

for i in range(len(nodes)):
    plt.scatter(nodes[i][0],nodes[i][1])
    
plt.show()

