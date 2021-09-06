import numpy as np
import cv2
import cv2.aruco as aruco
cap = cv2.VideoCapture('output1.avi')
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
parameters =  aruco.DetectorParameters_create()
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

    # Display the resulting frame
    cv2.imshow('frame',frame_markers)
    cv2.waitKey(50)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


