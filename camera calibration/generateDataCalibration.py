import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import os

#These values do not work
"""
Camera matrix is 
 [[663.37480487   0.         323.38546471]
 [  0.         663.37480487 222.7211238 ]
 [  0.           0.           1.        ]]

along with distortion coefficients :
 [[ 5.29412278e+00]
 [-1.31680153e+02]
 [-8.76018657e-03]
 [ 7.11994834e-04]
 [ 4.40783204e+02]
 [ 5.14189993e+00]
 [-1.30160326e+02]
 [ 4.36826627e+02]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]
 [ 0.00000000e+00]]
"""

mtx_prev = np.array([[663.37480487, 0, 323.38546471],[0, 663.37480487, 222.7211238],[0,0,1]])
dist_prev = np.array([ [-1.31680153e+02],[-8.76018657e-03],[ 7.11994834e-04],[ 4.40783204e+02],
                       [ 5.14189993e+00],[-1.30160326e+02],[ 4.36826627e+02],[ 0.00000000e+00],
                       [ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],[ 0.00000000e+00],
                       [ 0.00000000e+00]])

calib = False

def read_chessboards(images):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    return allCorners,allIds,imsize

def calibrate_camera(allCorners,allIds,imsize):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[ 1000.,    0., imsize[0]/2.],
                                 [    0., 1000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5,1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      charucoCorners=allCorners,
                      charucoIds=allIds,
                      board=board,
                      imageSize=imsize,
                      cameraMatrix=cameraMatrixInit,
                      distCoeffs=distCoeffsInit,
                      flags=flags,
                      criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors



workdir = os.getcwd()
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(7, 5, 1, .8, aruco_dict)
#imboard = board.draw((2000, 2000))
#test = cv2.imwrite("chessboard.jpg", imboard)
#fig = plt.figure()
#ax = fig.add_subplot(1,1,1)
#plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
#ax.axis("off")
#plt.show()


os.chdir("cameraCalibration\data")
datadir = os.getcwd()
print(os.listdir(datadir))

images = np.array([f for f in os.listdir(datadir) if f.endswith(".png") ])

order = np.argsort([int(p.split(".")[-2][5:]) for p in images])
images = images[order]


if calib == True:
    allCorners,allIds,imsize=read_chessboards(images)
    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners,allIds,imsize)
    print("Camera matrix is \n", mtx, "\nalong with distortion coefficients : \n", dist)

else:
    i=20 # select image id
    plt.figure()
    frame = cv2.imread(images[i])
    img_undist = cv2.undistort(frame,mtx_prev,dist_prev,None)
    plt.subplot(1,2,1)
    plt.imshow(frame)
    plt.title("Raw image")
    plt.axis("off")
    plt.subplot(1,2,2)
    plt.imshow(img_undist)
    plt.title("Corrected image")
    plt.axis("off")
    plt.show()

