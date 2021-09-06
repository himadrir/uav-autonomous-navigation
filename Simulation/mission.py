import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
connection_string = 'tcp:127.0.0.1:5763'
baudrate = 57600
#--- Now that we have started the SITL and we have the connection string (basically the ip and udp port)...
def readmission(aFileName):
    """
    Load a mission from a file into a list.

    This function is used by upload_mission().
    """
    print "Reading mission from file: %s\n" % aFileName
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist
def upload_mission(aFileName):
        """
        Upload a mission from a file.
        """
        #Read mission from file
        missionlist = readmission(aFileName)

        #print "\nUpload mission from a file: %s" % import_mission_filename
        #Clear existing mission from vehicle
        print ' Clear mission'
        cmds = vehicle.commands
        cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print ' Upload mission'
        vehicle.commands.upload()

print(">>>> Connecting with the UAV <<<")
vehicle = connect(connection_string, baud = baudrate, wait_ready=True)     #- wait_ready flag hold the program untill all the parameters are $
upload_mission('third.waypoints')
time.sleep(2)

vehicle.parameters['ARMING_CHECK'] = 1
while not vehicle.is_armable:	
	print " Waiting for vehicle to initialise..."
	time.sleep(1)
vehicle.mode = VehicleMode("GUIDED")
#time.sleep(5)
vehicle.armed = True
vehicle.flush()
while not vehicle.armed:
	time.sleep(1)
	print("waiting")

#vehicle.flush()
#vehicle.mode = VehicleMode("GUIDED")
#vehicle.flush()
print(vehicle.mode.name)
time.sleep(2)
#vehicle.commands.takeoff(2)
#print("this line")
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

	
#for i in range(10):
#	print(vehicle.mode.name)
#	time.sleep(1)
vehicle.mode = VehicleMode("LAND")
vehicle.flush()
time.sleep(2)
print(vehicle.mode.name)
time.sleep(10)

vehicle.close()
#vehicle.armed = False
