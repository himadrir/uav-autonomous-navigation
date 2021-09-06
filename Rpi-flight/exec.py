from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

connection_string = 'tcp:127.0.0.1:5763'
#connection_string = '/dev/ttyS0'
baudrate = 57600

vehicle = connect(connection_string, baud = baudrate, wait_ready=True)
uav_location = vehicle.location.global_relative_frame
print "UAV Location    Lat = %.7f  Lon = %.7f"%(uav_location.lat, uav_location.lon)
vehicle.close()
