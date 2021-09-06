"""
This code assumes that images used for calibration are of the same arUco marker board provided with code

"""

import cv2
from cv2 import aruco
import yaml
import numpy as np
import time

# For validating results, show aruco board to camera.
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_4X4_1000)

#Provide length of the marker's side
markerLength = 150  # Here, measurement unit is centimetre.

arucoParams = aruco.DetectorParameters_create()

camera = cv2.VideoCapture('output3.avi')
ret, img = camera.read()


with open('calibration_logitech_cam.yaml') as f:
    loadeddict = yaml.load(f)
mtx = loadeddict.get('camera_matrix')
dist = loadeddict.get('dist_coeff')
mtx = np.array(mtx)
dist = np.array(dist)

ret, img = camera.read()
img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
h,  w = img_gray.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

pose_r = []
pose_t = []
count = 0

while True:
    ret, img = camera.read()
    img_aruco = img
    im_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    im_gray = cv2.adaptiveThreshold(im_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,255,0)
    ret,im_gray = cv2.threshold(im_gray,70,255,cv2.THRESH_BINARY)
    h,  w = im_gray.shape[:2]
    dst = cv2.undistort(im_gray, mtx, dist, None, newcameramtx)
    ids = None
    corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, aruco_dict, parameters=arucoParams)
        #cv2.imshow("original", img_gray)
    try:
        if ids == None or len(ids)>1:
            print ("pass1")
        elif ids[0] == 0:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)
            print ("Rotation ", rvec, "Translation", tvec)
            print(tvec.item(0),tvec.item(1))

            img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
            img_aruco = aruco.drawAxis(img_aruco, newcameramtx, dist, rvec, tvec, 10)    # axis length 100
            #im_gray = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
        else:
            print("pass2")
    except:
        print("hi")
    
    cv2.imshow("World co-ordinate frame axes", img_aruco)
    cv2.waitKey(50)
    #time.sleep(4)
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break
        
cv2.destroyAllWindows()

