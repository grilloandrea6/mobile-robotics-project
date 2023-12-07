# Mobile Robotics Project
# Local Navigation Submodule
# Author: Badil Mujovi
# 2023, fall semester

import numpy as np
import cv2
from math import dist
import pyvisgraph as vg

BIG_VALUE = 100000

# Basic preprocessing done on the image to get the shape of the obstacles
def prepareImage(frame, threshold = 100):
    kernel = np.ones((2,2),np.uint8)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   
    
    # Noise reduction 
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = cv2.medianBlur(gray, 3)

    _ ,thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Morphological transfsormations to get the outline of an object
    # Done by taking the difference between the dilated and eroded img.
    # We also apply a dilation to make the outline bigger.
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_GRADIENT, kernel)
    thresh = cv2.dilate(thresh, kernel, iterations = 5)
        
    return thresh

# Simplyfing contours by considering two points and deleting all the points
# vetween them that are inside a mergin epsilon
def approx_contour(contour, epsilon = 10.0, closed = True):
    # Delete unnecessary points within a margin on a straight line
    contour_approx = cv2.approxPolyDP(contour, epsilon = epsilon, closed = closed)
    return contour_approx

# The scaling of the contours is done by taking the sum of the vectors
# given by the two adjacent points to a vertice and moving it in the resulting
# direction scaled by the length of the thymio in pixels
def scalePoints(points, length):
    for p in range(points.shape[0]):
        
        scalingDirection = np.array([0,0])

        if p == points.shape[0]-1:
            vect1 = (points[p, 0, :]- points[0, 0, :])/np.linalg.norm((points[p, 0, :]- points[0, 0, :]),2)
            vect2 = (points[p, 0, :]- points[p-1, 0, :])/np.linalg.norm((points[p, 0, :]- points[p-1, 0, :]))
            scalingDirection = vect1 + vect2

        else:
            vect1 = (points[p, 0, :]- points[p+1, 0, :])/np.linalg.norm((points[p, 0, :]- points[p+1, 0, :]),2)
            vect2 = (points[p, 0, :]- points[p-1, 0, :])/np.linalg.norm((points[p, 0, :]- points[p-1, 0, :]))
            scalingDirection = vect1 + vect2
        
        scalingDirection = scalingDirection/np.linalg.norm(scalingDirection,2)
        points[p, 0, :] = points[p,0,:] + (scalingDirection*length).astype(int)

    return points

# Delete unnecessary points from contour and scale it to take Thymio's
# size into account
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
            
            contour_scaled = scalePoints(contour_approx, 11.5/scalingFactor)

            # All the points that have been scaled outside of the ROI are given
            # a big value so that the optimal path finding does not consider them
            for point in contour_scaled:
                y_max = img.shape[1]
                x_max = img.shape[0]

                if point[0, 0] > y_max:
                    point[0,0] = BIG_VALUE
                elif point[0,0] < 0:
                    point[0,0] = -BIG_VALUE
                
                if point[0,1] > x_max:
                    point[0,1] = BIG_VALUE
                elif point[0,1] < 0: 
                    point[0,1] = -BIG_VALUE
            
            listContour.append(contour_scaled[:,0,:])
            cv2.drawContours(img, contour_scaled, -1, (0,0,255), 10)
    else:
        print("No contour found")
    
    return img, listContour


def addStartAndGoal(contourList, start, goal, scalingFactor):
    start = np.array(([(start/scalingFactor)]),np.int32)
    goal = np.array(([(goal/scalingFactor)]),np.int32)
    
    # We add the goal and start points 
    contourList.append(goal) # Goal
    contourList.insert(0, start)  # Start
    
    #print(contourList)
    return contourList

# Finds the shortest path using Djikstra's algorithm
def findShortestPath(contourList):
    polyList = list()
    poly = list()   
    
    # Create the list of points to be used for visibility graph.
    # The obstacles are considered to be polygons.
    for obstacle in contourList[:]:
        polyList = list()
        for i in range(len(obstacle)):
            polyList.append(vg.Point(obstacle[i,0], obstacle[i,1]))
            
        poly.append(polyList)

    # Create the visibillity graph and find optimal path
    g = vg.VisGraph()
    g.build(poly[:], status = False)
    shortest = g.shortest_path(vg.Point(contourList[0][0,0], contourList[0][0,1]), vg.Point(contourList[-1][0,0], contourList[-1][0,1]))

    # Convert optimal path to np arrays
    shortestPath = list()
    for p in shortest:
        shortestPath.append(np.array([p.x, p.y]))
    path = np.array(shortestPath).astype(int)
    
    return path