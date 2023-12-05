import numpy as np
import cv2
from math import dist
import pyvisgraph as vg

def prepareImage(frame, threshold = 100):
    kernel = np.ones((2,2),np.uint8)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   
    
    # Noise reduction 
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = cv2.medianBlur(gray, 3)

    _ ,thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Morphological transfsormations to get the outline of an object
    # Done by taking the difference between the dilated img and itself.
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_GRADIENT, kernel)
    thresh = cv2.dilate(thresh, kernel, iterations = 5)
        
    return thresh

def approx_contour(contour, epsilon = 10.0, closed = True):
    # Delete unnecessary points within a margin on a straight line
    contour_approx = cv2.approxPolyDP(contour, epsilon = 10.0, closed = True)
    return contour_approx

def contourCenter(contour):
    M = cv2.moments(contour)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    center = [cX, cY]
    return center

def getLength(p1, p2):
    length = dist(p1, p2)
    return length

def getDilatationScale(d1, d2):
    return d1/d2

def scalePointsFromCenter(points, center, length):
    for p in range(points.shape[0]):
        distCenter = getLength(center, [points[p,0,0], points[p,0,1]])
        scale = getDilatationScale(distCenter+length, distCenter)
        
        # Translate to center
        points[p, 0, :] = points[p, 0, :] - center 
        # Scale
        points[p, 0, :] = points[p,0,:]*scale
        # Translate back to center
        points[p, 0, :] = points[p, 0, :] + center
    return points

def drawSimplifiedContours(contours, img, scalingFactor):
    listContour = list()
    if len(contours) > 0:
        for cont in contours:
            # We try to find the small contours to delete them
            # by looking at the smallest square area that they can fill
            [_ , (height, width), _] = cv2.minAreaRect(cont)
            area = height*width
            if area < 2500: # equivalent to a square of 50x50 px
                continue
            
            # Approximation on contour into few points and scaling of those points
            contour_approx = approx_contour(cont)
            
            # There's a problem with the center calculation
            contour_scaled = scalePointsFromCenter(contour_approx, contourCenter(contour_approx), 10/scalingFactor)
            
            listContour.append(contour_scaled[:,0,:])
            cv2.drawContours(img, contour_scaled, -1, (0,0,255), 10)
    else:
        print("No contour found")
    
    return img, listContour

def addStartAndGoal(contourList, start, goal, scalingFactor):
    #AG - not needed - y, x = img.shape[0:2]
    #print(contourList)
    start = np.array(([(start/scalingFactor)]),np.int32)
    goal = np.array(([(goal/scalingFactor)]),np.int32)
    
    # We add the goal and start points 
    contourList.append(goal) # Goal
    contourList.insert(0, start)  # Start
    
    #print(contourList)
    return contourList

def findShortestPath(contourList):
    polyList = list()
    poly = list()   
    
    #Goal is not appended right in the poly list
    for obstacle in contourList[:]:
        polyList = list()
        for i in range(len(obstacle)):
            polyList.append(vg.Point(obstacle[i,0], obstacle[i,1]))
            
        poly.append(polyList)

    #print(poly)
    g = vg.VisGraph()
    g.build(poly[:], status = False)
    shortest = g.shortest_path(vg.Point(contourList[0][0,0], contourList[0][0,1]), vg.Point(contourList[-1][0,0], contourList[-1][0,1]))

    shortestPath = list()
    for p in shortest:
        shortestPath.append(np.array([p.x, p.y]))
    path = np.array(shortestPath).astype(int)
    
    return path