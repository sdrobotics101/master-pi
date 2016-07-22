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
MOTOR_SERVER_IP = "10.0.0.46"

def start_transitions(current, previous):
    print("in start_transitions")
    previous = "Start"
    return("GateDeadRecon", current, previous)

def kill_transitions(current, previous):
    print("in kill_transitions")
    time.sleep(.5)
    if previous == "Kill":
           return("IsKilled", current, previous)
    
    previous = "Kill"
    return("Start", current, previous)

def iskilled_transitions(current, previous):
    print("in iskilled_transitions")
    data, active = client.getRemoteBufferContents(MOTOR_KILL,MOTOR_SERVER_IP,MOTOR_SERVER_ID)
    isKilled = Unpack(Kill, data)
    if isKilled == True:
#       When "Kill" calls "IsKilled", if it gets a return of
#       previous = "Kill", the robot is still killed, but
#       if the return of previous = "IsKilled", the robot
#       is no longer killed, and transitions to "Start".
       return("Kill", current, "Kill")

    returnto = previous
    previous = "IsKilled"
    return(returnto, current, previous)

def gatedr_transitions(current, previous):
    print("in gatedr_transitions")
    time.sleep(1)
    previous = "GateDeadRecon"
    return("GateDeadRecon", current, previous)

def gatevision_transitions(current, previous):
    print("in gatevision_transitions")
    previous = "GateVision"
    return("GateVision", current, previous)

def pathfinder_transitions(current, previous):
    print("in pathfinder_transitions")
    previous = "PathFinder"
    return("PathFinder", current, previous)


if __name__== "__main__":
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
    m.add_state("GateDeadRecon", gatedr_transitions)
    m.add_state("GateVision", gatevision_transitions)
    m.add_state("PathFinder", pathfinder_transitions)
    m.add_state("Error", None, end_state=1)

    m.set_start("Kill")
    m.run("Kill")

