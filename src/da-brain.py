import sys
sys.path.insert(0, '../DistributedSharedMemory/build')
import pydsm

from statemachine import StateMachine

def start_transitions(current):
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
    client = pydsm.Client(42, 60, True)
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
    
