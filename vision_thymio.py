"""
Code written by Badil Mujovi

Project for the EPFL course Basics of Mobile Robotics.

2023, fall semester
"""

import numpy as np
import cv2
import global_navigation as gn
import math

### Markers detection functions
class Vision_Thymio(object):

    def __init__(self, camera_index):
        print("Init Vision Thymio")
        self.cap = self.startVideoCapture(camera_index)
        self.dict = self.initDictAruco()
    
    # We use IDs: 
    # - 1 for the pos and direction of the thymio
    # - 2 for the goal
    # - 3, 4, 5, and 6 to get the scaling factor and perspective transformation matrix
    
    def initDictAruco(self,type = cv2.aruco.DICT_4X4_50):
        dict = cv2.aruco.getPredefinedDictionary(type)
        return dict

    # Return the id and corners of the markers found
    def detectArucoMarkers(self, img):
        marker_ids = []
        corners = []
        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Corners are returned in clockwise order starting with top left
        corners, ids, _ = cv2.aruco.detectMarkers(image=img, dictionary=self.dict)
        
        if len(corners) != 0:
            for i in range(len(ids)):
                marker_ids.append(ids[i][0])
                    
        return marker_ids, corners


    # Corrected image acquisition with camera Matrix
    def getFrame(self):
        
        # These are the values used to correct different types of opticals aberations
        # such as spherical or tangential aberation. 
        # These values have been found after following a calibration process
        # (opencv camera calibration with a ChAruco board)
        
        mtx = np.array([[989.7401734,  0.,           653.72226958],
                        [0.,           998.64802591, 347.61109886],
                        [0.,           0.,           1.          ]])
        dist = np.array([[ 0.12258701, -0.76974406, -0.00790144,  0.00456124,  1.16348775]])
        
        _, frame = self.cap.read()
        
        h,  w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

    # We compute a scaling factor by dividing the distance between
    # two Arcuo Markers measured in px by the distance in meters.

    # The perspective transformation is found by takeing 4 corners of the ROI
    # We can then map our image to a new one with corrected perspective.
    def getPerspectiveAndScaling(self, width_roi=48.4, height_roi=31):
        
        # Need to measure the size of the ROI in real life
        ratio_vh = width_roi/height_roi
        transformMatrix = []
        
        # loops until good transform matrix found 
        while len(transformMatrix) == 0:
            img = self.getFrame()
            ids, corners = self.detectArucoMarkers(img)
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

                # Previous ROI
                inputPoints = np.array([pointA, pointB, pointC, pointD],np.float32)
                
                # Dimensions of new ROI with same aspect ratio
                hmax = img.shape[1]
                wmax = hmax*ratio_vh
                
                # New ROI
                outputPoints = np.array([[0, 0],
                                [0, hmax],
                                [wmax, hmax],
                                [wmax, 0]], np.float32)

                transformMatrix = cv2.getPerspectiveTransform(inputPoints, outputPoints)
                sizeROI = np.array([hmax, wmax]).astype(int)
                scalingFactor = height_roi/hmax
            else:
                transformMatrix = []
                
                print("Can't find markers 3, 4, 5 and 6 for perspective transformation")
                print("Found ids", ids)

        self.transformMatrix, self.sizeROI, self.scalingFactor = transformMatrix, sizeROI, scalingFactor

    # Return image with corrected perspective
    def getCorrectedImage(self):
        img = self.getFrame()
        origImg = img
        img = cv2.warpPerspective(img, self.transformMatrix ,(self.sizeROI[1], self.sizeROI[0]),flags=cv2.INTER_LINEAR)

        return img, origImg

    # Detect marker with id 2 as goal position
    def getGoalPosition(self, ids, corners): 
        position = []
        found = False
        
        if (len(ids) != 0) and (2 in ids):
            found = True
            mark = corners[ids.index(2)][0]

            # Position is the mean value of each corner of the marker
            position = (np.sum(mark, 0)/4).astype(int)

        return found, position*self.scalingFactor if found else position

    # Detects marker with id 1 as thymio position
    def getThymioPos(self, ids, corners):
        position = []
        orientation = []
        pose = ()
        found = False

        # If some aruco patterns are detected, verify if 1 is among them
        if len(ids) != 0 and 1 in ids:
            found = True
            mark = corners[ids.index(1)][0]

            # Position is the mean value of each corner of the marker
            position = (np.sum(mark, 0)/4).astype(int)
            position = (position*self.scalingFactor)
   
            # Vector for the direction of the thymio (from the two left corners of the marker)
            orientation = mark[0,:]-mark[-1,:]
            orientation = orientation.astype(int)
            orientation = math.atan2(orientation[1],orientation[0])

            pose = position, orientation

        return found, pose # AG - , img

    # ========================================================

    # Used to control the camera feed
    def startVideoCapture(self,camera_index):
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        cap.set(3,854)
        cap.set(4,480)
        return cap

    def stopVideoCapture(self):
        self.cap.release()
        cv2.destroyAllWindows()
        
    # ========================================================

    # Need to find the visibility graph
    def getVisibilityGraph(self, img):
        thresh = gn.prepareImage(img)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame, contourList = gn.drawSimplifiedContours(contours, img, self.scalingFactor)

        return contourList, frame

    # Returns the optimal path as an array and draws it on the image
    def getOptimalPath(self, start, staticObstacleList, goal, img):
        path = []
                
        staticObstacleList = gn.addStartAndGoal(staticObstacleList, start, goal, self.scalingFactor)

        path = gn.findShortestPath(contourList = staticObstacleList)
        
        # AG - adapt it to be run in jupyter notebook
        for i in range(len(path)-1):
            img = cv2.line(img, path[i, :].astype(int), path[i+1, :].astype(int), color=(0, 0, 255), thickness=2)

        path = path * self.scalingFactor

        return path, img #*self.scalingFactor

