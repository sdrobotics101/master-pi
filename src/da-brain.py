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

def start_transitions(current, previous):
    print("Resetting Position and Starting...")
    sensorreset = SensorReset()
#this is setting the x, y, and z axes to 0
    sensorreset.pos[0] = 0
    sensorreset.pos[1] = 0
    sensorreset.pos[2] = 0
#this toggles the boolean change   
    sensorreset.reset = not sensorreset.reset
#this packs it all up
    client.setLocalBufferContents(MASTER_SENSOR_RESET,Pack(sensorreset))

    previous = "Start"
    return("GateDeadReckon", current, previous)

def kill_transitions(current, previous):
    print("Checking if the robot is killed...")
    time.sleep(.5)
    killPrevious = "Kill"
#keeps on checking whether the robot is killed or not
    if previous == "Kill":
           return("IsKilled", current, killPrevious)
    
    return("Start", current, previous)

def iskilled_transitions(current, previous):
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
       return("Kill", current, "Kill")

    returnto = previous
    previous = "IsKilled"
    return(returnto, current, previous)

def gatedr_transitions(current, previous):
    print("Navigating to Gate using Dead Reckoning...") 
    time.sleep(.5)
    gateDRPrevious = "GateDeadReckon"

#step 1: check if the robot is killed
    if previous == "Start":
       return("IsKilled", current, gateDRPrevious)

#step 2: check if forward vision has feedback    
    elif previous == "IsKilled":
       return("GateVisionFeedback", current, gateDRPrevious)

#step 3: check if downward vision has feedback
    elif previous == "GateVisionFeedback":
       return("PathFinder", current, gateDRPrevious)

#step 4: set velocity in m/s
    controlinput = ControlInput()
#setting angular position in the x,y, and z axes 
#with pos[0]=position and pos[1]=time it takes
    controlinput.angular[0].pos[0] = 0
    controlinput.angular[0].pos[1] = 0
    controlinput.angular[1].pos[0] = 0
    controlinput.angular[1].pos[1] = 0
    controlinput.angular[2].pos[0] = 0
    controlinput.angular[2].pos[1] = 0

#setting linear velocity in the x, y, and z axes
    controlinput.linear[0].vel = 3
    controlinput.linear[1].vel = 3
    controlinput.linear[2].vel = 0

#setting the mode, with lin(z,y,x) and ang(z,y,x)
    controlinput.mode = 39
    
    client.setLocalBufferContents(MASTER_CONTROL,Pack(controlinput))

    return("GateDeadReckon", current, previous)

def gatevisionfeed_transitions(current, previous):
    print("Does Vision Have Feedback?")
    gateVFPrevious = "GateVisionFeedback"
        
    data, active = client.getRemoteBufferContents(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)    
    seeGate = Unpack(Location, data)
#this is for testing purposes only.  Remove later.
    seeGate.confidence = 127

    if seeGate.confidence >= 128:
       print("Vision has Feedback!")
       return("GateVision", current, gateVFPrevious)

    print("Vision doesn't have feedback :(")
    return("GateDeadReckon", current, gateVFPrevious)

def gatevision_transitions(current, previous):
    print("Orienting Robot to Gate with Vision...")
    time.sleep(.5)
    gateVPrevious = "GateVision"

    if previous == "GateVisionFeedback" or previous == "IsKilled":
           return("IsKilled", current, gateVPrevious)

    return("GateVision", current, gateVPrevious)

def pathfinder_transitions(current, previous):
    print("in pathfinder_transitions")
    pathPrevious = "PathFinder"

    data, active = client.getRemoteBufferContents(TARGET_LOCATION,DOWNWARD_VISION_SERVER_IP,DOWNWARD_VISION_SERVER_ID)
    seePath = Unpack(Location, data)
#this is for testing purposes only.  Remove later.
    seePath.confidence = 128

    if seePath.confidence >= 128:
       print("The path has been spotted!")
       return("Error", current, pathPrevious)

    print("No path spotted :(")
    return("GateDeadReckon", current, pathPrevious)


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
    m.add_state("Error", None, end_state=1)

    m.set_start("Kill")
    m.run("Kill")

