import time
from vision_thymio import *

# create object
vis = Vision_Thymio()
time.sleep(1) #make sure that everything is loaded

# init of the perspective and scaling of the image
vis.getPerspectiveAndScaling()

# get obstacles on the map
img = vis.getCorrectedImage()
staticObstacleList = vis.getVisibilityGraph(img)

input("place goal and thymio on the map")
foundGoal = foundThymio = False
goalPos = []
thymioPos = []

# try ten times to get thymio and goal position on the map, if not print error and exit
for i in range(10):
    img = vis.getCorrectedImage()
    ids,corners = vis.detectArucoMarkers(img)

    if not foundGoal:
        foundGoal, goalPos = vis.getGoalPosition(ids, corners)
    if not foundThymio:
        foundThymio, thymioPos = vis.getThymioPos(ids,corners) 

    if foundGoal and foundThymio:
        break
    
    time.sleep(0.1)

if not (foundGoal and foundThymio):
    print("ERROR - cannot found goal or thymio")
    exit()

print(f"found thymio and goal at iteration {i}")

print(f"position of the goal: {goalPos} position of the thymio: {thymioPos}")


path = vis.getOptimalPath(thymioPos[0], staticObstacleList, goalPos)

print(path)

## Only this last function is needed to recompute the optimal path
#path = [(924,567),(214,511)]
#print(path)
#
#import sys
#import math
#from local_navigation import Local_Navigation
#local_nav = Local_Navigation(path)
#x = []
#y = []
#theta = 0
while(True):
    img = vis.getCorrectedImage()
    ids,corners = vis.detectArucoMarkers(img)

    foundThymio, thymioPos = vis.getThymioPos(ids,corners) 

    
    if not foundThymio:
        print("empty position!!!")
    else:
        #print(pos, orient, math.atan2(orient[1],orient[0]))
        print(thymioPos)
        print(thymioPos[1] * 180 / math.pi)

        #x.append(pose[0])
        #y.append(pose[1])
        #theta = pose[2]
        #v,w = local_nav.path_follow((x[-1],y[-1],theta))
        #wl,wr= local_nav.differential_steering(v,w)
        #print(f"differential steering {(wl,wr)}") 
        #input()

        #print(f"my pose is {pose}")
        #v,w = local_nav.path_follow((x[-1],y[-1],theta))

    time.sleep(0.2)
    
    
    #for i in range(len(path)-1):
    #    img = cv2.line(img, path[i], path[i+1], color=(0, 0, 255), thickness=2) 
    
    # Display the resulting frame
    #cv2.imshow('frame',img)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

# When everything done, release the capture
stopVideoCapture(cap)