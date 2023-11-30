import math
from local_navigation import Local_Navigation
import matplotlib.pyplot as plt 

nodes = [(0,0),(1,0),(3,10),(5,5),(10,2),(12,7)]



local_nav = Local_Navigation()

local_nav.define_path(nodes)

x = [0]
y = [0]
theta = 25 * math.pi / 180

for i in range(1000):
    print(f"iterazione numero {i}")
    v,w = local_nav.path_follow((x[-1],y[-1],theta))
    v = v/7.5
    wl,wr= local_nav.differential_steering(v,w)
    print(f"differential steering {(wl,wr)}") 
    if abs(wl) > 400 or abs(wr) > 400:
        print(f"SATURATED")
        #input()
    if (v,w) == (-1,-1):
        print("FINISHED")
        break
    x.append(x[-1] + v *  math.cos(theta)/5000)
    y.append( y[-1] + v * math.sin(theta)/5000)
    theta = theta - w/400
    

    plt.arrow(x[-1],y[-1],v*math.cos(theta)/2000,v*math.sin(theta)/2000)

for i in range(len(nodes)):
    plt.scatter(nodes[i][0],nodes[i][1])
    
plt.show()

