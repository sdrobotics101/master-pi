import sys
sys.path.insert(0, '../DistributedSharedMemory/build')
sys.path.insert(0, '../PythonSharedBuffers/src')
from Constants import *
import pydsm
import time
import math
from ctypes import *
from Sensor import *
from Master import *
from Navigation import *
from Vision import *
from Serialization import *
from Quaternions import *
from statemachine import StateMachine
import numpy as np

def kill_transitions(cargo, previous):
	print("Checking if the robot is killed...")
	time.sleep(.5)
#keeps on checking whether the robot is killed or not
	return("IsKilled", cargo)


def iskilled_transitions(cargo, previous):
	print("Is the robot killed?")
	data, active = client.getRemoteBufferContents(MOTOR_KILL,MOTOR_SERVER_IP,MOTOR_SERVER_ID)
	killObject = Unpack(Kill, data)
#this is for testing purposes only, remove later 
	# killObject = Kill()
	# killObject.isKilled = False
	# active = True

	print(killObject.isKilled)

	if killObject.isKilled == True:
		print("The robot is killed")
		return("Kill", cargo)

	elif previous == "Kill":
		print("The robot is not killed, starting...")
		return("Start", cargo)
	elif previous == "KILL":
		print("The robot is not killed, starting...")
		return ("Start", cargo)
	return(previous, cargo)


def start_transitions(cargo, previous):
	print("Resetting Position and Starting...")
	sensorreset = SensorReset()

#this sets the x,y,and z axes to 0, resetting the pool coordinates at the dock
	sensorreset.pos[xaxis] = 0
	sensorreset.pos[yaxis] = 0
	sensorreset.pos[zaxis] = 0

#this toggles the boolean change
	sensorreset.reset = not sensorreset.reset

#this packs it all up
	client.setLocalBufferContents(MASTER_SENSOR_RESET,Pack(sensorreset))

	return("GateDeadReckon", cargo)


def gatedr_transitions(cargo, previous):
	print("Navigating to Gate using Dead Reckoning...")
	time.sleep(.5)

#step 0: broadcast goals to all other pis
	goals = Goals()
	goals.forwardVision = GOAL_FIND_GATE
	goals.downwardVision = GOAL_FIND_PATH
	goals.sonar = GOAL_NONE

	client.setLocalBufferContents(MASTER_GOALS,Pack(goals))

#step 1: check if the robot is killed
	if previous == "Start" or previous == "GateDeadReckon" or previous == "GateVision":
		return("IsKilled", cargo)

#step 2: check if forward vision has feedback    
	elif previous == "IsKilled":
		return("GateVisionFeedback", cargo)

#step 3: check if downward vision has feedback
	elif previous == "GateVisionFeedback":
		return("PathFinder", cargo)

#step 4: set velocity in m/s
	controlinput = ControlInput()

#setting angular Position
	controlinput.angular[xaxis].pos[POSITION] = 0
	controlinput.angular[xaxis].pos[TIME] = 0
	controlinput.angular[yaxis].pos[POSITION] = 0
	controlinput.angular[yaxis].pos[TIME] = 0
	controlinput.angular[zaxis].pos[POSITION] = 0
	controlinput.angular[zaxis].pos[TIME] = 0

#setting linear velocity
	controlinput.linear[xaxis].vel = XVEL
	controlinput.linear[yaxis].vel = YVEL
	controlinput.linear[zaxis].vel = ZVEL

#setting the mode, with lin(z,y,x) and ang(z,y,x)
	controlinput.mode = 39

	client.setLocalBufferContents(MASTER_CONTROL,Pack(controlinput))

	return("GateDeadReckon", cargo)


def gatevisionfeed_transitions(cargo, previous):
	print("Does Vision Have Feedback?")
 
	buff, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
	active = 1 #TODO testing var
	seeGate = LocationArray()
	if active:
		SeeGate = Unpack(LocationArray, buff)
#this is for testing purposes only.  Remove later.
		seeGate.locations[0].confidence = 128
		seeGate.locations[1].confidence = 128

		if seeGate.locations[0].confidence >= CONFIDENCE and seeGate.locations[1].confidence >= CONFIDENCE:
			print("Vision has Feedback!")
			return("GateVision", cargo)

	print("Vision doesn't have feedback :(")
	return("GateDeadReckon", cargo)


def gatevision_transitions(cargo, previous):
	print("Orienting Robot to Gate with Vision...")
	time.sleep(.5)

	data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
	pose, ret = client.getRemoteBufferContents(SENSORS_ANGULAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
	pointLocation  = Unpack(LocationArray, data)
	poolPosition = Unpack(Angular, pose)
	active = 1 #testing var
	# pointLocation = LocationArray()
	# poolPosition = Angular()

#testing variables
	# poolPosition.pos[QUAT_W] = 1
	# poolPosition.pos[QUAT_X] = 1
	# poolPosition.pos[QUAT_Y] = 1
	# poolPosition.pos[QUAT_Z] = 1

	controlinput = ControlInput()
	if active:
		quaternion = (
		poolPosition.pos[QUAT_W],
		poolPosition.pos[QUAT_X],
		poolPosition.pos[QUAT_Y],
		poolPosition.pos[QUAT_Z])
		euler = test_q2e(quaternion)
		roll = euler[xaxis]
		pitch = euler[yaxis]
		yaw = euler[zaxis]

#step 1: check if the robot is killed
	if previous == "GateVisionFeedback" or previous == "GateVision":
		return("IsKilled", cargo)

#step 2: check if the downward camera has located the path
	elif previous == "IsKilled":
		return("PathFinder", cargo)

	pointLocation.locations[0].loctype = GATEPOLE #testing variable
	active = 1

	if active:
		if previous == "PathFinder" and pointLocation.locations[0].loctype == GATEPOLE:
			print("Orienting robot to gate...")
			gateX = (pointLocation.locations[0].x + pointLocation.locations[1].x)/2
			gateY = (pointLocation.locations[0].y + pointLocation.locations[1].y)/2
			gateZ = (pointLocation.locations[0].z + pointLocation.locations[1].z)/2
			#testing
			# gateX = 1
			# gateY = 1
			# gateZ = 1

			controlinput.angular[xaxis].pos[POSITION] = 0
			controlinput.angular[xaxis].pos[TIME] = 0
			controlinput.angular[yaxis].pos[POSITION] = 0
			controlinput.angular[yaxis].pos[TIME] = 0
			controlinput.angular[zaxis].pos[POSITION] = math.atan(gateY/gateX)
			controlinput.angular[zaxis].pos[TIME] = 0
 
			matYaw = np.array([[math.cos(-yaw),-math.sin(-yaw)],[math.sin(-yaw),math.cos(-yaw)]])
			matPos = np.array([[gateX],[gateY]])
			matPos = np.dot(matYaw, matPos)
			# This velocity value is changable, it is setting the total velocity when going towards the gate
			V = 1
 
			controlinput.linear[xaxis].vel = math.sqrt(pow(V, 2) - math.pow(matPos[1,0], 2))
			controlinput.linear[yaxis].vel = matPos[1,0]
			controlinput.linear[zaxis].pos[POSITION] = 0
			controlinput.linear[zaxis].pos[TIME] = 0
			return("GateVision", cargo)

	print("Orientatinon failed.")
	return("GateDeadReckon", cargo)


def pathfinder_transitions(cargo, previous):
	print("in pathfinder_transitions")

	data, active = client.getRemoteBufferContents(TARGET_LOCATION,DOWNWARD_VISION_SERVER_IP,DOWNWARD_VISION_SERVER_ID)
	seePath = Unpack(LocationAndRotation, data)
	#this is for testing purposes only.  Remove later.
	active = 1
	# seePath = LocationAndRotation
	# seePath.confidence = 127

	if active:
		if ord(seePath.confidence) >= CONFIDENCE:
			print("The path has been spotted!")
			return("PathOrientation", cargo)

	print("No path spotted :(")
	return(previous, cargo)


def pathorient_transitions(cargo, previous):
	print("Orienting robot to path...")
	time.sleep(.5)
	data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
	pose, ret = client.getRemoteBufferContents(SENSORS_ANGULAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
	orient = Unpack(LocationAndRotation, data)
	poolPosition = Unpack(Angular, pose)

#for testing
	active = 1
	# orient = LocationAndRotation()
	# poolPosition = Angular()

#testing var
	# poolPosition.pos[QUAT_W] = 0
	# poolPosition.pos[QUAT_X] = 0
	# poolPosition.pos[QUAT_Y] = 0
	# poolPosition.pos[QUAT_Z] = 0

	controlinput = ControlInput()

#step 0.5 Converting from quaternion to euler pool coordinates
	if active:
		quaternion = (
		poolPosition.pos[QUAT_W],
		poolPosition.pos[QUAT_X],
		poolPosition.pos[QUAT_Y],
		poolPosition.pos[QUAT_Z])
		euler = test_q2e(quaternion)
		roll = euler[xaxis]
		pitch = euler[yaxis]
		yaw = euler[zaxis]

	if active:
		print("Correcting course...")
		orientX = orient.xrot
		orientY = orient.yrot
		orientZ = orient.zrot
		controlinput.angular[xaxis].pos[POSITION] = 0
		controlinput.angular[xaxis].pos[TIME] = 0
		controlinput.angular[yaxis].pos[POSITION] = 0
		controlinput.angular[yaxis].pos[TIME] = 0
		controlinput.angular[zaxis].pos[POSITION] = yaw + orientZ
		controlinput.angular[zaxis].pos[TIME] = 0
	return("BuoyDeadReckon", cargo)

def buoydr_transitions(cargo, previous):
	print("Dead Reckoning towards buoys...")
	time.sleep(.5)

#step 0: broadcast goals to all other pis
	goals = Goals()
	goals.forwardVision = GOAL_FIND_RED_BUOY
	goals.downwardVision = GOAL_FIND_PATH
	goals.sonar = GOAL_NONE

	client.setLocalBufferContents(MASTER_GOALS,Pack(goals))

#step 1: check if the robot is killed
	if previous == "PathOrientation" or previous == "BuoyDeadReckon":
		return("IsKilled", cargo)

#step 2: check if the robot sees the red buoy
	elif previous == "IsKilled":
		return("CheckRed", cargo)

#step 3: set velocity in m/s
	controlinput = ControlInput()

#setting angular Position
	controlinput.angular[xaxis].pos[POSITION] = 0
	controlinput.angular[xaxis].pos[TIME] = 0
	controlinput.angular[yaxis].pos[POSITION] = 0
	controlinput.angular[yaxis].pos[TIME] = 0
	controlinput.angular[zaxis].pos[POSITION] = 0
	controlinput.angular[zaxis].pos[TIME] = 0

#setting linear velocity
	controlinput.linear[xaxis].vel = XVEL
	controlinput.linear[yaxis].vel = YVEL
	controlinput.linear[zaxis].vel = ZVEL

#setting the mode, with lin(z,y,x) and ang(z,y,x)
	controlinput.mode = 39

	client.setLocalBufferContents(MASTER_CONTROL,Pack(controlinput))

	return("BuoyDeadReckon", cargo)


def checkred_transitions(cargo, previous):
	print("Checking if camera sees a red buoy...")

	data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
	seeRed = Unpack(Location, data)

#this is for testing only remove later
	active = 1
	# seeRed = Location()
	# seeRed.confidence = 128

	if active:
		if seeRed.confidence >= CONFIDENCE:
			print("We have a visual!")
			return("BuoyVision", cargo)

	return("BuoyDeadReckon", cargo)


def checkyellow_transitions(cargo, previous):
	print("Looking for the yellow buoy...")
	time.sleep(.5)
	data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
	pose, ret = client.getRemoteBufferContents(SENSORS_ANGULAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
	seeYellow = Unpack(Location, data)
	poolPosition = Unpack(Angular, pose)

#for testing
	active = 1
	# seeYellow = Location()
	# poolPosition = Angular()

#step 0: broadcast goals to all other pis
	goals = Goals()
	goals.forwardVision = GOAL_FIND_YELLOW_BUOY
	goals.downwardVision = GOAL_FIND_PATH
	goals.sonar = GOAL_NONE

	client.setLocalBufferContents(MASTER_GOALS,Pack(goals))

#testing var
	poolPosition.pos[QUAT_W] = 0
	poolPosition.pos[QUAT_X] = 0
	poolPosition.pos[QUAT_Y] = 0
	poolPosition.pos[QUAT_Z] = 0

	controlinput = ControlInput()

#testing var
	seeYellow.confidence = 128
	seeYellow.loctype = YELLOW

#step 0.5 Converting from quaternion to euler pool coordinates
	if active:
		quaternion = (
		poolPosition.pos[QUAT_W],
		poolPosition.pos[QUAT_X],
		poolPosition.pos[QUAT_Y],
		poolPosition.pos[QUAT_Z])
		euler = test_q2e(quaternion)
		roll = euler[xaxis]
		pitch = euler[yaxis]
		yaw = euler[zaxis]

#step 1 If the confidence level is met, correct course
	if active:
		if ord(seeYellow.confidence) >= CONFIDENCE and seeYellow.loctype == YELLOW:
			print("We have a visual! Correcting course...")
			yellowX = seeYellow.x
			yellowY = seeYellow.y
			yellowZ = seeYellow.z
			controlinput.angular[xaxis].pos[POSITION] = 0
			controlinput.angular[xaxis].pos[TIME] = 0
			controlinput.angular[yaxis].pos[POSITION] = 0
			controlinput.angular[yaxis].pos[TIME] = 0
			controlinput.angular[zaxis].pos[POSITION] = math.atan(yellowY/yellowX)
			controlinput.angular[zaxis].pos[TIME] = 0
 
			matYaw = np.array([[math.cos(-yaw),-math.sin(-yaw)],[math.sin(-yaw),math.cos(-yaw)]])
			matPos = np.array([[yellowX],[yellowY]])
			matPos = np.dot(matYaw, matPos)
#		This velocity value is changable, it is setting the total velocity when going towards the gate   
			V = 1
 
			controlinput.linear[xaxis].vel = math.sqrt(pow(V, 2) - math.pow(matPos[1,0], 2))
			controlinput.linear[yaxis].vel = matPos[1,0]
			controlinput.linear[zaxis].pos[POSITION] = 0
			controlinput.linear[zaxis].pos[TIME] = 0
			return("BuoyVision", cargo)

	print("no visual :(")
	return("BouyDeadReckon", cargo)


def buoyvision_transitions(cargo, previous):
	print("Moving to yellow Buoy...")
	time.sleep(.5)

#step 1: Check if the robot is killed
	if previous == "CheckRed" or previous == "BouyVision" or previous == "CheckYellow":
		return("IsKilled", cargo)

#step 2: Check/navigate to the yellow buoy
	elif previous == "IsKilled":
		return("CheckYellow", cargo)

	return("BuoyVision", cargo)


if __name__== "__main__":
	print("Initializing Client")
	client = pydsm.Client(MASTER_SERVER_ID, 60, True)

	print("Creating Local Buffers")
	client.registerLocalBuffer(MASTER_CONTROL,sizeof(ControlInput),False)
	client.registerLocalBuffer(MASTER_GOALS,sizeof(Goals),False)
	client.registerLocalBuffer(MASTER_SENSOR_RESET,sizeof(SensorReset),False)
	time.sleep(1)
	controlinput = ControlInput()
	goals = Goals()
	sensorreset = SensorReset()
	client.setLocalBufferContents(MASTER_CONTROL,Pack(controlinput))
	client.setLocalBufferContents(MASTER_GOALS,Pack(goals))
	client.setLocalBufferContents(MASTER_SENSOR_RESET,Pack(sensorreset))
	# this is setting the initial sensorreset.reset value to false
	sensorreset = SensorReset()
	sensorreset.reset = False

	print("Creating Remote Buffers")
	client.registerRemoteBuffer(SENSORS_LINEAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
	client.registerRemoteBuffer(SENSORS_ANGULAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
	client.registerRemoteBuffer(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
	client.registerRemoteBuffer(TARGET_LOCATION,DOWNWARD_VISION_SERVER_IP,DOWNWARD_VISION_SERVER_ID)
	client.registerRemoteBuffer(TARGET_LOCATION,SONAR_SERVER_IP,SONAR_SERVER_ID)
	client.registerRemoteBuffer(MOTOR_KILL,MOTOR_SERVER_IP,MOTOR_SERVER_ID)
	time.sleep(1)

	print("Creating State Machine")
	m = StateMachine()

	m.add_state("Start", start_transitions)
	m.add_state("Kill", kill_transitions)
	m.add_state("IsKilled", iskilled_transitions)
	m.add_state("GateDeadReckon", gatedr_transitions)
	m.add_state("GateVisionFeedback", gatevisionfeed_transitions)
	m.add_state("GateVision", gatevision_transitions)
	m.add_state("PathFinder", pathfinder_transitions)
	m.add_state("PathOrientation", pathorient_transitions)
	m.add_state("BuoyDeadReckon", buoydr_transitions)
	m.add_state("CheckRed", checkred_transitions)
	m.add_state("CheckYellow", checkyellow_transitions)
	m.add_state("BuoyVision", buoyvision_transitions)
	m.add_state("Error", None, end_state=1)
	m.add_state("EndOfRun", None, end_state=1)

	m.set_start("Kill")
	m.run("PLACEHOLDER")

