
import math
from vision_thymio import *
#====================================================================
#====================================================================
#====================================================================

# initialize variables
cap = startVideoCapture()
dict = initDictAruco()

time.sleep(2)

# get values for image correction (persepctive and scaling)
transformMatrix, sizeROI, scalingFactor = getPerspectiveAndScaling(cap, dict)

# Then we need to do the computing fot visibility graph and optimal path
# We can compute the visibility graph first and find the static obstacles
#staticObstacleList = getVisibilityGraph(cap, transformMatrix, sizeROI)

# Then we get the goal position
#goal = getGoalPosition(cap, dict, transformMatrix, scalingFactor, sizeROI)

# Finally, we compute the optimal path
#path = getOptimalPath(cap, transformMatrix, dict, staticObstacleList, goal, sizeROI, scalingFactor)
# Only this last function is needed to recompute the optimal path
path = [(924,567),(214,511)]
print(path)

import sys
import math
from local_navigation import Local_Navigation
local_nav = Local_Navigation(path)
x = []
y = []
theta = 0
while(True):
    
    # Capture frame-by-frame
    img = getCorrectedImage(cap, transformMatrix, sizeROI)
    
    
    # How to get the thymios position
    ids, corners = detectArucoMarkers(img, dict)
    pos, orient, _ = getThymioPos(img, ids, corners, scalingFactor)
    
    if len(pos) == 0:
        print("empty position!!!")
    else:
        print(f"scalingFactor {scalingFactor}")
        #print(pos, orient, math.atan2(orient[1],orient[0]))
        pose = (pos[0]/scalingFactor,pos[1]/scalingFactor,math.atan2(orient[1],orient[0]))
        x.append(pose[0])
        y.append(pose[1])
        theta = pose[2]
        v,w = local_nav.path_follow((x[-1],y[-1],theta))
        wl,wr= local_nav.differential_steering(v,w)
        print(f"differential steering {(wl,wr)}") 
        input()

        print(f"my pose is {pose}")
        #v,w = local_nav.path_follow((x[-1],y[-1],theta))


    
    
    for i in range(len(path)-1):
        img = cv2.line(img, path[i], path[i+1], color=(0, 0, 255), thickness=2) 
    
    # Display the resulting frame
    cv2.imshow('frame',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
stopVideoCapture(cap)