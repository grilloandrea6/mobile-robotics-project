import sys
import math
sys.path.append('../')
from local_navigation import Local_Navigation
import matplotlib.pyplot as plt 

nodes = [(0,0),(5,0),(10,5),(3,3),(11,11),(-5,-10),(-5,-10)]


local_nav = Local_Navigation(nodes)
print("created object")

pose = []
pose.append((0,0,45* math.pi / 180))
x = [0]
y = [0]

for i in range(1000):
    print(f"iterazione numero {i}")
    v,w = local_nav.path_follow(pose[i])

    wl,wr= local_nav.differential_steering(v,w)
    print(f"differential steering {(wl,wr)}") 
    if abs(wl) > 3000 or abs(wr) > 3000:
        print(f"SATURATED")
        #break

    if (v,w) == (-1,-1):
        print("FINISHED")
        break
    pose.append(   (pose[-1][0] + v * math.cos(pose[-1][2])/400, pose[-1][1] + v * math.sin(pose[-1][2])/400, pose[-1][2] + w/400) )
    x = pose[-1][0]
    y = pose[-1][1]
    theta = pose[-1][2]
    

    plt.arrow(x,y,v*math.cos(theta)/600,v*math.sin(theta)/600)
for i in range(len(nodes)):
    plt.scatter(nodes[i][0],nodes[i][1])
plt.show()

