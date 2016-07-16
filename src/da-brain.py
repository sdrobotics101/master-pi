import sys
import constants
sys.path.insert(0, '../DistributedSharedMemory/build')
import pydsm
import RPi.GPIO as GPIO
import time
MAG_SWITCH_GPIO = 17

from statemachine import StateMachine

def start_transitions(current):
    while(GPIO.input(MAG_SWITCH_GPIO)==0):
        time.sleep(1)
    print("in start_transitions")
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
    
    client.registerLocalBuffer(MASTER_CONTROL,49,False)
    client.registerLocalBuffer(MASTER_GOALS,3,False)
    
    client.registerRemoteBuffer(SENSORS_LINEAR,SENSOR_SERVER_ID,SENSOR_SERVER_IP)
    client.registerRemoteBuffer(SENSORS_ANGULAR,SENSOR_SERVER_ID,SENSOR_SERVER_IP)
    client.registerRemoteBuffer(TARGET_LOCATION,FORWARD_VISION_SERVER_ID,FORWARD_VISION_SERVER_IP)
    client.registerRemoteBuffer(TARGET_LOCATION,DOWNWARD_VISION_SERVER_ID,DOWNWARD_VISION_SERVER_IP)
    client.registerRemoteBuffer(TARGET_LOCATION,SONAR_SERVER_ID,SONAR_SERVER_IP)

    
/*struct ControlInput 
{union AxisControl 
    {double vel;
        float pos[2];
    } 
    angular[3], linear[3];
    uint8_t mode;
} 
_controlInput;*/

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MAG_SWITCH_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
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
    
