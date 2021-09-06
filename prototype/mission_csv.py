import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import csv

csvfile = open('dummy.csv') 
reader = csv.DictReader(csvfile)

#connection_string = 'tcp:127.0.0.1:5763'
connection_string = '/dev/ttyS0'
baudrate = 57600
#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...
def add_mission(reader):
   
	'''Adds a takeoff command and four waypoint commands to the current mission. 
	The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

	The function assumes vehicle.commands matches the vehicle mission state 
	(you must have called download at least once in the session and after clearing the mission)
		
	'''
	cmds = vehicle.commands

	print " Clear any existing commands"
	cmds.clear() 
    
	print " Define/add new commands."
	# Add new commands. The meaning/order of the parameters is documented in the Command class. 
  
	#Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 4))

	#Define the MAV_CMD_NAV_WAYPOINT locations and add the commands
	for row in reader:
		cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, float(row['lat']), float(row['lng']), 4
		))   

	print " Upload new commands to vehicle"
	cmds.upload()

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string,baud=baudrate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are $
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
aTargetAltitude = 3
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
	try:
		nextwaypoint=vehicle.commands.next
		print("nextwaypoint:",nextwaypoint)
		if(nextwaypoint==4):
			break
	except KeyboardInterrupt:
		break
	

	

#	print(vehicle.mode.name)
#	time.sleep(1)
vehicle.mode = VehicleMode("LAND")
vehicle.flush()
time.sleep(2)
print(vehicle.mode.name)
time.sleep(10)

vehicle.close()
csvfile.close()
#vehicle.armed = False


