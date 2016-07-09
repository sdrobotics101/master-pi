from statemachine import StateMachine

def start_transitions(current):
    return ("Start", current)
    
def gatewatch_transitions(current):
    return ("Gatewatch", current)
    
def move1_transitions(current):
    return ("Move1m", current)
    
def visionmove_transitions(current):
    return ("VisionMove", current)
    
def sonarmove_transitions(current):
    return ("SonarMove", current)
    
def avgmove_transitions(current):
    return ("AverageMove", current)
    
def passgate_transitions(current):
    return ("PassThruGate", current)
    
def pathfinder_transitions(current):
    return ("PathFinder", current)

if __name__== "__main__":
    m = StateMachine()
    m.add_state("Start", start_transitions)
    m.add_state("Gatewatch", gatewatch_transitions)
    m.add_state("Move1m", move1_transitions)
    m.add_state("VisionMove", visionmove_transitions)
    m.add_state("SonarMove", sonarmove_transitions)
    m.add_state("AverageMove", avgmove_transitions)
    m.add_state("PassThruGate", passgate_transitions)
    m.add_state("PathFinder", pathfinder_transitions)
    m.set_start("Start")
    