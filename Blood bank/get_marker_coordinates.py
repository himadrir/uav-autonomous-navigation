"""

NOTE: be sure to be using the latest dronekit. 
sudo pip uninstall dronekit
sudo pip uninstall pymavlink

cd dronekit-python
git pull



"""
from os import sys, path
import cv2
from cv2 import aruco
import time
import math
import yaml
import numpy as np

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
#from opencv.lib_aruco_pose import *

# For validating results, show aruco board to camera.
aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_4X4_1000)

#Provide length of the marker's side
markerLength = 150  # Here, measurement unit is centimetre.

arucoParams = aruco.DetectorParameters_create()

#connection_string = 'tcp:127.0.0.1:5763'
connection_string = '/dev/ttyS0'
baudrate = 57600

vehicle = connect(connection_string, baud = baudrate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are $    
#--------------------------------------------------
#-------------- FUNCTIONS  
#--------------------------------------------------    

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a Location object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt and `is_relative` values
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
    print "dlat, dlon", dLat, dLon

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return(newlat, newlon)

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam): #changed
    x_uav = x_cam
    y_uav = -y_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)
        

#--------------------------------------------------
#-------------- PARAMETERS  
#-------------------------------------------------- 
rad_2_deg   = 180.0/math.pi
deg_2_rad   = 1.0/rad_2_deg 

#--------------------------------------------------
#-------------- LANDING MARKER  
#--------------------------------------------------    
#--- Define Tag
id_to_find      = 0
marker_size     = 150 #- [cm]
freq_send       = 1 #- Hz

land_alt_cm         = 50.0
angle_descend       = 20*deg_2_rad
land_speed_cms      = 30.0



#--- Get the camera calibration path
# Find full directory path of this script, used for loading config and other files
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
                
                
time_0 = time.time()

while True:                
    try:
        ret, img = camera.read()
        img_aruco = img
        im_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        im_gray = cv2.adaptiveThreshold(im_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,255,0)
        ret,im_gray = cv2.threshold(im_gray,70,255,cv2.THRESH_BINARY)
        h,  w = im_gray.shape[:2]
        dst = cv2.undistort(im_gray, mtx, dist, None, newcameramtx)
        ids = None
        corners, ids, rejectedImgPoints = aruco.detectMarkers(dst, aruco_dict, parameters=arucoParams)
        uav_location = vehicle.location.global_relative_frame
        
        if ids == None or len(ids)>1:
            print ("pass1")
        elif ids[0] == 0:
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)
            print ("Rotation ", rvec, "Translation", tvec)
            x_cm = tvec.item(0)
            y_cm = tvec.item(1)
            x_cm, y_cm = camera_to_uav(x_cm, y_cm)
            if uav_location.alt >= 5.0:
                    z_cm = uav_location.alt*100.0
            else:
                z_cm = tvec.item(2)
            angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)    
            if time.time() >= time_0 + 1.0/freq_send:
                time_0 = time.time()
                # print ""
                print " "
                print "Altitude = %.0fcm"%z_cm
                print "Marker found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x*rad_2_deg, angle_y*rad_2_deg)
                
                north, east = uav_to_ne(x_cm, y_cm, vehicle.attitude.yaw)
                print "Marker N = %5.0f cm   E = %5.0f cm   Yaw = %.0f deg"%(north, east, vehicle.attitude.yaw*rad_2_deg)
                
                marker_lat, marker_lon  = get_location_metres(uav_location, north*0.01, east*0.01)  
                #-- If angle is good, descend

                if check_angle_descend(angle_x, angle_y, angle_descend):
                    print "Low error: descending"
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt-(land_speed_cms*0.01/freq_send))
                else:
                    location_marker         = LocationGlobalRelative(marker_lat, marker_lon, uav_location.alt)
                    
                #vehicle.simple_goto(location_marker)
                print "UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon)
                print "Commanding to   Lat = %.7f  Lon = %.7f"%(location_marker.lat, location_marker.lon)
        else:
            print("pass2")
    except:
        print("exception triggered")


        #--- COmmand to land
        #if z_cm <= land_alt_cm:
        #    if vehicle.mode == "GUIDED":
        #        print (" -->>COMMANDING TO LAND<<")
        #        vehicle.mode = "LAND"
            
