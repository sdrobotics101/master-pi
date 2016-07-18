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

def start_transitions(current):
    print("in start_transitions")
    while(not client.doesRemoteExist(MOTOR_KILL,"10.0.1.2",MOTOR_SERVER_ID)):   
        time.sleep(1)
    data.active = client.getRemoteBufferContents(MOTOR_KILL,"10.0.1.2",MOTOR_SERVER_ID)
    kill = Kill()
    Unpack(Kill,data)
    while(kill,isKilled and active):
        print("MURDURIZED")
        data,active = client.getRemoteBufferContents(MOTOR_KILL,"10.0.1.2",MOTOR_SERVER_ID)
        time.sleep(1)
     return ("Start", current)
    
def gatewatch_transitions(current):
    print("in gatewatch_transitions")
    return ("Gatewatch", current)
    
def move1_transitions(current):
    print("in move1_transitions")
    return ("Move1m", current)
    
def visionmove_transitions(current):
    print("in visionmove_transitions")
    return ("VisionMove", current)
    
def sonarmove_transitions(current):
    print("in sonarmove_transitions")
    return ("SonarMove", current)
    
def avgmove_transitions(current):
    print("in avgmove_transitions")
    return ("AverageMove", current)
    
def passgate_transitions(current):
    print("in passgate_transitions")
    return ("PassThruGate", current)
    
def pathfinder_transitions(current):
    print("in pathfinder_transitions")
    return ("PathFinder", current)
    
if __name__== "__main__":
    client = pydsm.Client(MASTER_SERVER_ID, 60, True)
    
    print("Creating Local Buffers")    
    client.registerLocalBuffer(MASTER_CONTROL,sizeof(ControlInput),False)
    client.registerLocalBuffer(MASTER_GOALS,sizeof(Goals),False)
    client.registerLocalBuffer(MASTER_SENSOR_RESET,sizeof(SensorReset),False)
    
    print("Creating Remote Buffers")
    client.registerRemoteBuffer(SENSORS_LINEAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
    client.registerRemoteBuffer(SENSORS_ANGULAR,SENSOR_SERVER_IP,SENSOR_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION,FORWARD_VISION_SERVER_IP,FORWARD_VISION_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION,DOWNWARD_VISION_SERVER_IP,DOWNWARD_VISION_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION,SONAR_SERVER_IP,SONAR_SERVER_ID)
    client.registerRemoteBuffer(MOTOR_KILL,MOTOR_SERVER_IP,MOTOR_SERVER_ID)    

    
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
    m.add_state("Gatewatch", gatewatch_transitions)
    m.add_state("Move1m", move1_transitions)
    m.add_state("VisionMove", visionmove_transitions)
    m.add_state("SonarMove", sonarmove_transitions)
    m.add_state("AverageMove", avgmove_transitions)
    m.add_state("PassThruGate", passgate_transitions)
    m.add_state("PathFinder", pathfinder_transitions)
    m.add_state("Error", None, end_state=1)
    
    m.set_start("Start")
    m.run("PLACEHOLDER")
    
