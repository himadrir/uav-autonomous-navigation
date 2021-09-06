import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import csv
import cv2
from cv2 import aruco
import yaml
import numpy as np

########################################ARUCO CALIB########################################
# For validating results, show aruco board to camera.
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_6X6_1000)

#Provide length of the marker's side
markerLength = 10  # Here, measurement unit is centimetre.

arucoParams = aruco.DetectorParameters_create()

camera = cv2.VideoCapture(0)
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

############################################################################################

csvfile = open('dummy.csv') 
reader = csv.DictReader(csvfile)

connection_string = 'tcp:127.0.0.1:5763'
baudrate = 57600
#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...
def add_mission(reader):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print " Clear any existing commands"
    cmds.clear() 
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the MAV_CMD_NAV_WAYPOINT locations and add the commands
    for row in reader:
    	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(row['lat']), float(row['lng']), 75))   

    print " Upload new commands to vehicle"
    cmds.upload()

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are $
#upload_mission('third.waypoints')
add_mission(reader)
time.sleep(2)

vehicle.parameters['ARMING_CHECK'] = 1
while not vehicle.is_armable:	
	print " Waiting for vehicle to initialise..."
	time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
vehicle.flush()
while not vehicle.armed:
	time.sleep(1)
	print("waiting")


print(vehicle.mode.name)
time.sleep(2)
aTargetAltitude = 1
print "Taking off!"
vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
while True:
	print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
	if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
		print "Reached target altitude"
		break
time.sleep(1)

print "\nstarting mission"
vehicle.commands.next=0

vehicle.mode = VehicleMode("AUTO")
while True:
	nextwaypoint=vehicle.commands.next
        print("nextwaypoint:",nextwaypoint)
	if(nextwaypoint==6):
	     break	

#vehicle.mode = VehicleMode("LOITER")

print "\ndetecting image"
while True:
    ret, img = camera.read()
    img_aruco = img
    im_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    h,  w = im_gray.shape[:2]
    dst = cv2.undistort(im_gray, mtx, dist, None, newcameramtx)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, aruco_dict, parameters=arucoParams)
    if ids == None:
        print ("pass")
    else:
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)
        print ("Rotation ", rvec, "Translation", tvec)
        img_aruco = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))
        img_aruco = aruco.drawAxis(img_aruco, newcameramtx, dist, rvec, tvec, 10)    # axis length 100 can be changed according to your requirement
    cv2.imshow("World co-ordinate frame axes", img_aruco)
    cv2.waitKey(50)
    if cv2.waitKey(0) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

vehicle.mode = VehicleMode("LAND")
vehicle.flush()
time.sleep(2)
print(vehicle.mode.name)
time.sleep(10)

vehicle.close()
csvfile.close()



