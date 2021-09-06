import numpy as np
import cv2

cap = cv2.VideoCapture(0)
path = './photos/'
count = 0    
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    #cv2.imshow('frame',frame)
    #if cv2.waitKey(1) & 0xFF == ord('c'):
    #    cv2.imwrite(path+str(count)+'.png',frame)cv2.imwrite(path+str(count)+'.png',frame)cv2.imwrite(path+str(count)+'.png',frame)
    #    count = count + 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.waitKey(500)
    cv2.imwrite(path+str(count)+'.png',frame)
    count = count + 1
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
