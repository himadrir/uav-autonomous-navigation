import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
connection_string = "/dev/ttyS0"
baudrate = 57600
#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud = baudrate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are $
vehicle.parameters['ARMING_CHECK'] = 0

time.sleep(5)
vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = True

while not vehicle.armed:
	time.sleep(1)
	print("waiting")

time.sleep(10)
vehicle.armed = False

