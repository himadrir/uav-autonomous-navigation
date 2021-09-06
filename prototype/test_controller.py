import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

connection_string = '/dev/ttyS0'
baudrate = 57600
#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string,baud = baudrate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are $
try:
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
	aTargetAltitude = 2
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
	vehicle.mode = VehicleMode("LAND")
	vehicle.flush()
	time.sleep(2)
	print(vehicle.mode.name)
	time.sleep(3)

	vehicle.close()
except KeyboardInterrupt:
	vehicle.mode = VehicleMode("STABILIZE")
	print(vehicle.mode.name)
	vehicle.armed = False
	vehicle.flush()
	time.sleep(3)
	vehicle.close()


