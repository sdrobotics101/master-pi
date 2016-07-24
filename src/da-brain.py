import sys
sys.path.insert(0, '../DistributedSharedMemory/build')
sys.path.insert(0, '../PythonSharedBuffers/src')
from Constants import *
import pydsm
import time
from ctypes import *
from Sensor import *
from Master import *
from Navigation import *
from Vision import *
from Serialization import *

from statemachine import StateMachine
#MOTOR_SERVER_IP = "10.0.0.46"

def start_transitions(cargo, previous):
    print("Resetting Position and Starting...")
    sensorreset = SensorReset()
#this is setting the x, y, and z axes to 0
    sensorreset.pos[xaxis] = 0
    sensorreset.pos[yaxis] = 0
    sensorreset.pos[zaxis] = 0
#this toggles the boolean change   
    sensorreset.reset = not sensorreset.reset
#this packs it all up
    client.setLocalBufferContents(MASTER_SENSOR_RESET,Pack(sensorreset))

    return("GateDeadReckon", cargo)

def kill_transitions(cargo, previous):
    print("Checking if the robot is killed...")
    time.sleep(.5)
#keeps on checking whether the robot is killed or not
    if previous == "Kill":
           return("IsKilled", cargo)
    
    return("Start", cargo)

def iskilled_transitions(cargo, previous):
    print("Is the robot killed?")
    data, active = client.getRemoteBufferContents(MOTOR_KILL,MOTOR_SERVER_IP,MOTOR_SERVER_ID)
    killObject = Unpack(Kill, data)
#this is for testing purposes only, remove later 
    killObject.isKilled = False
    active = True

    print(killObject.isKilled)
    if killObject.isKilled == True or active == False:
#       When "Kill" calls "IsKilled", if it gets a return of
#       previous = "Kill", the robot is still killed, but
#       if the return of previous = "IsKilled", the robot
#       is no longer killed, and transitions to "Start".
       return("Kill", cargo)

    return(previous, cargo)

def gatedr_transitions(cargo, previous):
    print("Navigating to Gate using Dead Reckoning...") 
    time.sleep(.5)

#step 1: check if the robot is killed
    if previous == "Start" or previous == "GateDeadReckon":
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
    controlinput.linear[xaxis].vel = 3
    controlinput.linear[yaxis].vel = 3
    controlinput.linear[zaxis].vel = 0

#setting the mode, with lin(z,y,x) and ang(z,y,x)
    controlinput.mode = 39
    
    client.setLocalBufferContents(MASTER_CONTROL,Pack(controlinput))

    return("GateDeadReckon", cargo)

def gatevisionfeed_transitions(cargo, previous):
    print("Does Vision Have Feedback?")
        
    data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)    
    seeGate = Unpack(Location, data)
#this is for testing purposes only.  Remove later.
    seeGate.confidence = 128

    if seeGate.confidence[0] >= 128:
       print("Vision has Feedback!")
       return("GateVision", cargo)

    print("Vision doesn't have feedback :(")
    return("GateDeadReckon", cargo)

def gatevision_transitions(cargo, previous):
    print("Orienting Robot to Gate with Vision...")
    time.sleep(.5)

    if previous == "GateVisionFeedback":
       return("IsKilled", cargo)
    
    elif previous == "IsKilled":
       return("PathFinder", cargo) 

    return("GateDeadReckon", cargo)

def pathfinder_transitions(cargo, previous):
    print("in pathfinder_transitions")

    data, active = client.getRemoteBufferContents(TARGET_LOCATION,DOWNWARD_VISION_SERVER_IP,DOWNWARD_VISION_SERVER_ID)
    seePath = Unpack(Location, data)
#this is for testing purposes only.  Remove later.
    seePath.confidence = 127

    if seePath.confidence[0] >= 128:
       print("The path has been spotted!")
       return("PathOrientation", cargo)

    print("No path spotted :(")
    return("GateDeadReckon", cargo)

def pathorient_transitions(cargo, previous):
    print("Orienting robot to path...")
    time.sleep(.5)
    return("BuoyDeadReckon", cargo)

def buoydr_transitions(cargo, previous):
    print("Dead Reckoning towards buoys...")
    time.sleep(.5)

    if previous == "PathOrientation" or previous == "BuoyDeadReckon":
       return("IsKilled", cargo)

    elif previous == "IsKilled":
       return("CheckRed", cargo)

    return("BuoyDeadReckon", cargo)

def checkred_transitions(cargo, previous):
    print("Checking if camera sees a red buoy...")

    data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)    
    seeRed = Unpack(Location, data)

#this is for testing only remove later
    seeRed.confidence = 128
    if seeRed.confidence[0] >= 128:
       print("We have a visual!")
       return("BuoyVision", cargo)

    return("BuoyDeadReckon", cargo)

def checkyellow_transitions(cargo, previous):
    print("Looking for the yellow buoy...")

    data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)    
    seeYellow = Unpack(Location, data)

#for testing
    seeYellow.confidence = 127
    if seeYellow.confidence[0] >= 128:
       print("We have a visual! Correcting course...")
#      here is where we'll correct the orientation of the robot to go to yellow
       return("BuoyVision", cargo)

    print("no visual :(")
    return("BuoyVision", cargo)
    
def buoyvision_transitions(cargo, previous):
    print("Moving to yellow Buoy...")

    if previous == "CheckRed":
       return("IsKilled", cargo)

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
    # TODO perhaps initialize these with values first?
    client.setLocalBufferContents(MASTER_CONTROL,Pack(controlinput))
    client.setLocalBufferContents(MASTER_GOALS,Pack(goals))
    client.setLocalBufferContents(MASTER_SENSOR_RESET,Pack(sensorreset))
    #this is setting the initial sensorreset.reset value to false
    sensorreset = SensorReset()
    sensorreset.reset = False

    print("Creating Remote Buffers")
    client.registerRemoteBuffer(SENSORS_LINEAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
    client.registerRemoteBuffer(SENSORS_ANGULAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
    client.registerRemoteBuffer(MASTER_SENSOR_RESET,MASTER_SERVER_IP,MASTER_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION,DOWNWARD_VISION_SERVER_IP,DOWNWARD_VISION_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION,SONAR_SERVER_IP,SONAR_SERVER_ID)
    client.registerRemoteBuffer(MOTOR_KILL,MOTOR_SERVER_IP,MOTOR_SERVER_ID)
    time.sleep(1)


#struct ControlInput
#{union AxisControl
#    {double vel;
#       float pos[2];
#   }
#   angular[3], linear[3];
#   uint8_t mode;
#}
#_controlInput;
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

