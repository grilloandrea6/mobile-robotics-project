import numpy as np
import cv2
import time
import globalNavigation as gn
### Markers detection functions

# We use IDs: - 1 for the direction of the thymio
#             - 2 for the goal
#             - 3 and 4 to get the scaling factor of the distances by knowing the distance in meters
    
def initDictAruco(type = cv2.aruco.DICT_4X4_50):
    dict = cv2.aruco.getPredefinedDictionary(type)
    return dict

def detectArucoMarkers(img, dict):
    marker_ids = []
    corners = []
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Corners are given in clockwise order starting with top left 
    corners, ids, _ = cv2.aruco.detectMarkers(image=img, dictionary=dict)
    
    if len(corners) != 0:
        for i in range(len(ids)):
            marker_ids.append(ids[i][0])
                
    return marker_ids, corners


# Corrected image acquisition with camera Matrix
def getFrame(cap):
    
    # These are the values used to correct different types of opticals aberations
    # such as the spherical aberation or the tangential one
    # These values have been found after following a valibration process
    # (opencv camera calibration with charuco patterns)
    
    mtx = np.array([[989.7401734,    0.,         653.72226958],
            [  0.,         998.64802591, 347.61109886],
            [  0.,           0.,           1.        ]])
    dist = np.array([[ 0.12258701, -0.76974406, -0.00790144,  0.00456124,  1.16348775]])
    
    _, frame = cap.read()
    
    h,  w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

# We compute a scaling factor by dividing the distance
# between two Arcuo Markers measured in px and in meters

# The perspective transformation is found by takeing 4 corners of the ROI
# We can then map our image to a new one with corrected perspective
def getPerspectiveAndScaling(cap, dict, width_roi=48.4, height_roi= 31):
    
    # Need to measure the size of the ROI in real life
    ratio_vh = width_roi/height_roi
    transformMatrix = []
    
    # loops until good transform matrix found 
    while len(transformMatrix) == 0:
        _ , img = cap.read()
        ids, corners = detectArucoMarkers(img, dict)
        wmax = 0
        hmax = 0
        
        if ids != [] and (3 in ids) and (4 in ids) and (5 in ids) and (6 in ids):
            
            # Take the points of each marker closest to ROI
            mark4 = corners[ids.index(4)][0]
            mark3 = corners[ids.index(3)][0]
            mark5 = corners[ids.index(5)][0]
            mark6 = corners[ids.index(6)][0]
            
            pointA = mark4[2,:]
            pointB = mark3[1,:]
            pointC = mark5[0,:]
            pointD = mark6[3,:]

            # Warning, the point corresponding must be the closest to the  -->  to verify
            inputPoints = np.array([pointA, pointB, pointC, pointD],np.float32)

            
            hmax = img.shape[1]
            wmax = hmax*ratio_vh
            
            outputPoints = np.array([[0, 0],
                            [0, hmax],
                            [wmax, hmax],
                            [wmax, 0]], np.float32)
            
            
            img = cv2.circle(img, pointA.astype(int), radius=0, color= (0,0,255), thickness = 5)
            img = cv2.circle(img, pointB.astype(int), radius=0, color= (0,255,0), thickness = 5)
            img = cv2.circle(img, pointC.astype(int), radius=0, color= (255,0,0), thickness = 5)
            img = cv2.circle(img, pointD.astype(int), radius=0, color= (120,120,120), thickness = 5)
            
            # delete
            cv2.imshow("point for ROI", img)
            while True:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break 
            
            transformMatrix = cv2.getPerspectiveTransform(inputPoints, outputPoints)
            
            sizeROI = np.array([hmax, wmax]).astype(int)
            
            scalingFactor = height_roi/hmax
        else:
            transformMatrix = []
            print("Can't find markers 3,4, 5 and 6 for perspective transformation")

    return transformMatrix, sizeROI, scalingFactor

def getCorrectedImage(cap, transformMatrix, sizeROI):
    img = getFrame(cap)
    img = cv2.warpPerspective(img, transformMatrix ,(sizeROI[1], sizeROI[0]),flags=cv2.INTER_LINEAR)
    # Need to correct the perspective transformation 
    
    return img
    
    
def getGoalPosition(cap, dict, transformMatrix,scalingFactor, sizeROI):    
    position = []
    
    # If some aruco patterns are detected, verify if 2 are among them
    while len(position) == 0:
        img = getCorrectedImage(cap, transformMatrix, sizeROI)
        ids, corners = detectArucoMarkers(img, dict)

        input("Place marker 2 and press ENRER when ready")        
        
        if (len(ids) != 0) and (2 in ids):
            sum = [0,0]
            nElem = 0
            
            mark = corners[ids.index(2)][0]

            # Position is the mean value of each corner of the marker
            position = (np.sum(mark, 0)/4).astype(int)
        else:
            print("Can't find goal marker (2) !")
    
    return position*scalingFactor

def getThymioPos(img, ids, corners, scalingFactor):
        
    position = []
    orientation = []
    
    # If some aruco patterns are detected, verify if 1 are among them
    if len(ids) != 0 and 1 in ids:
            sum = [0,0]
            nElem = 0
            
            mark = corners[ids.index(1)][0]

            # Position is the mean value of each corner of the marker
            position = (np.sum(mark, 0)/4).astype(int)
            
            # Vector for the direction of the thymio (from two left corners of the mark)
            orientation = mark[0,:]-mark[-1,:]
            orientation = orientation.astype(int) 
            
            img = cv2.circle(img, position, radius = 0, color = (0,0,255), thickness=5)
            img = cv2.line(img,mark[0,:].astype(int), mark[-1,:].astype(int), color = (255,0,0), thickness=2)
            
            position = (position*scalingFactor)
            
    return position, orientation, img

# ========================================================

def startVideoCapture():
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(3,854)
    cap.set(4,480)
    return cap

def stopVideoCapture(cap):
    cap.release()
    cv2.destroyAllWindows()
    
# ========================================================

# Need to find the visibility graph without the goal and start positions
def getVisibilityGraph(cap, transformMatrix, sizeROI):
    
    img = getCorrectedImage(cap, transformMatrix, sizeROI)
    
    thresh = gn.prepareImage(img)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    frame, contourList = gn.drawSimplifiedContours(contours, img)
    
    # delete
    cv2.imshow("ContourList",frame)
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

    return contourList

def getOptimalPath(cap, transformMatrix, dict, staticObstacleList, goal, sizeROI, scalingFactor):
    path = []
    
    # We look at the initial position of the thymio to find the optimal path
    start = []
    while len(start) == 0 :
        
        img = getCorrectedImage(cap, transformMatrix, sizeROI)
        ids, corners = detectArucoMarkers(img, dict)
        
        input("Place the robot with marker 1 and press ENTER when ready!")
        start, _ , _ = getThymioPos(img, ids, corners, scalingFactor)
        
        if (len(start) == 0):
            print("Coudn't find the robot marker, retry !")

    _, staticObstacleList = gn.addStartAndGoal(img, staticObstacleList, start, goal, scalingFactor)

    path = gn.findShortestPath(contourList = staticObstacleList)
    
    for i in range(len(path)-1):
        img = cv2.line(img, path[i, :], path[i+1, :], color=(0, 0, 255), thickness=2)
    
    return path

