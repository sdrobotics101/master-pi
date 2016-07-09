from statemachine import StateMachine

if __name__== "__main__":
    m = StateMachine()
    m.add_state("Start", start_transitions)
    m.add_state("Gatewatch", gatewatch_transitions)
    m.add_state("Move1m", move1_transitions)
    m.add_state("VisionMove", visionmove_transitions)
    m.add_state("SonarMove", sonarmove_transitions)
    m.add_state("AverageMove", avgmove_transitions)
    m.add_state("PassThruGate, passgate_transitions)
    m.add_state("PathFinder", pathfinder_transitions)
    