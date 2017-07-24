import sys

sys.path.insert(0, '../DistributedSharedMemory/build')
sys.path.insert(0, '../PythonSharedBuffers/src')
import pydsm
import time
from Sensor import *
from Master import *
from Navigation import *
from Vision import *
from Serialization import *
from Quaternions import *
from statemachine import StateMachine
import numpy as np


color = ""


def kill_transitions(cargo, previous):
    print("Checking if the robot is killed...")
    time.sleep(.5)
    # keeps on checking whether the robot is killed or not
    return "IsKilled", cargo


def iskilled_transitions(cargo, previous):
    print("Is the robot killed?")
    data, active = client.getRemoteBufferContents(MOTOR_KILL, MOTOR_SERVER_IP, MOTOR_SERVER_ID)
    kill_object = Unpack(Kill, data)
    if kill_object.isKilled:
        print("The robot is killed")
        return "Kill", cargo
    elif previous == "Kill":
        print("The robot is not killed, starting...")
        return "Start", cargo
    elif previous == "KILL":
        print("The robot is not killed, starting...")
        return "Start", cargo
    return previous, cargo


def start_transitions(cargo, previous):
    print("Resetting Position and Starting...")
    sensor_reset = SensorReset()

    # this sets the x,y,and z axes to 0, resetting the pool coordinates at the dock
    sensor_reset.pos[xaxis] = 0
    sensor_reset.pos[yaxis] = 0
    sensor_reset.pos[zaxis] = 0

    # this toggles the boolean change
    sensor_reset.reset = not sensor_reset.reset

    # this packs it all up
    client.setLocalBufferContents(MASTER_SENSOR_RESET, Pack(sensor_reset))

    return "GateDeadReckon", cargo


def gatedr_transitions(cargo, previous):
    print("Navigating to Gate using Dead Reckoning...")
    time.sleep(.5)

    # step 1: broadcast goals to all other pis
    goals = Goals()
    goals.forwardVision = GOAL_FIND_GATE
    goals.downwardVision = GOAL_FIND_PATH
    goals.sonar = GOAL_NONE

    client.setLocalBufferContents(MASTER_GOALS, Pack(goals))

    # step 2: check if the robot is killed
    if previous == "Start" or previous == "GateDeadReckon" or previous == "GateVision":
        return "IsKilled", cargo
    # step 3: check if forward vision has feedback
    elif previous == "IsKilled":
        return "GateVisionFeedback", cargo
    # step 4: check if downward vision has feedback
    elif previous == "GateVisionFeedback":
        return "PathFinder", cargo

    # step 5: set velocity in m/s
    controlinput = ControlInput()

    # setting angular Position
    controlinput.angular[xaxis].pos[POSITION] = 0
    controlinput.angular[xaxis].pos[TIME] = 0
    controlinput.angular[yaxis].pos[POSITION] = 0
    controlinput.angular[yaxis].pos[TIME] = 0
    controlinput.angular[zaxis].pos[POSITION] = 0
    controlinput.angular[zaxis].pos[TIME] = 0

    # setting linear velocity
    controlinput.linear[xaxis].vel = XVEL
    controlinput.linear[yaxis].vel = YVEL
    controlinput.linear[zaxis].vel = ZVEL

    # setting the mode, with lin(z,y,x) and ang(z,y,x)
    controlinput.mode = 39

    client.setLocalBufferContents(MASTER_CONTROL, Pack(controlinput))

    return "GateDeadReckon", cargo


def gatevisionfeed_transitions(cargo, previous):
    print("Does Vision Have Feedback?")
    buff, active = client.getRemoteBufferContents(TARGET_LOCATION, FORWARD_VISION_SERVER_IP, FORWARD_VISION_SERVER_ID)
    see_gate = Unpack(LocationArray, buff)

    if see_gate.locations[0].confidence >= CONFIDENCE and see_gate.locations[1].confidence >= CONFIDENCE:
        print("Vision has Feedback!")
        return "GateVision", cargo

    print("Vision doesn't have feedback :(")
    return "GateDeadReckon", cargo


def gatevision_transitions(cargo, previous):
    print("Orienting Robot to Gate with Vision...")
    time.sleep(.5)
    data, active = client.getRemoteBufferContents(TARGET_LOCATION, FORWARD_VISION_SERVER_IP, FORWARD_VISION_SERVER_ID)
    pose, ret = client.getRemoteBufferContents(SENSORS_ANGULAR, SENSOR_SERVER_IP, SENSOR_SERVER_ID)
    point_loc = Unpack(LocationArray, data)
    pool_position = Unpack(Angular, pose)

    quaternion = (
        pool_position.pos[QUAT_W],
        pool_position.pos[QUAT_X],
        pool_position.pos[QUAT_Y],
        pool_position.pos[QUAT_Z])
    euler = test_q2e(quaternion)
    roll = euler[xaxis]
    pitch = euler[yaxis]
    yaw = euler[zaxis]

    # step 1: check if the robot is killed
    if previous == "GateVisionFeedback" or previous == "GateVision":
        return "IsKilled", cargo

    # step 2: check if the downward camera has located the path
    elif previous == "IsKilled":
        return "PathFinder", cargo

    if previous == "PathFinder" and point_loc.locations[0].loctype == GATEPOLE:
        print("Orienting robot to gate...")
        gate_x = (point_loc.locations[0].x + point_loc.locations[1].x) / 2
        gate_y = (point_loc.locations[0].y + point_loc.locations[1].y) / 2
        gate_z = (point_loc.locations[0].z + point_loc.locations[1].z) / 2

        control_input.angular[xaxis].pos[POSITION] = 0
        control_input.angular[xaxis].pos[TIME] = 0
        control_input.angular[yaxis].pos[POSITION] = 0
        control_input.angular[yaxis].pos[TIME] = 0
        control_input.angular[zaxis].pos[POSITION] = math.atan(gate_y / gate_x)
        control_input.angular[zaxis].pos[TIME] = 0

        mat_yaw = np.array([[math.cos(-yaw), -math.sin(-yaw)], [math.sin(-yaw), math.cos(-yaw)]])
        mat_pos = np.array([[gate_x], [gate_y]])
        mat_pos = np.dot(mat_yaw, mat_pos)
        # This velocity value is changeable, it is setting the total velocity when going towards the gate
        vel = VELOCITY

        control_input.linear[xaxis].vel = math.sqrt(pow(vel, 2) - math.pow(mat_pos[1, 0], 2))
        control_input.linear[yaxis].vel = mat_pos[1, 0]
        control_input.linear[zaxis].pos[POSITION] = 0
        control_input.linear[zaxis].pos[TIME] = 0
        return "GateVision", cargo

    print("Orientation failed.")
    return "GateDeadReckon", cargo


def pathfinder_transitions(cargo, previous):
    print("in pathfinder_transitions")

    data, active = client.getRemoteBufferContents(TARGET_LOCATION, DOWNWARD_VISION_SERVER_IP, DOWNWARD_VISION_SERVER_ID)
    see_path = Unpack(LocationAndRotation, data)

    if ord(see_path.confidence) >= CONFIDENCE:
        print("The path has been spotted!")
        return "PathOrientation", cargo

    print("No path spotted :(")
    return previous, cargo


def pathorient_transitions(cargo, previous):
    print("Orienting robot to path...")
    time.sleep(.5)
    data, active = client.getRemoteBufferContents(TARGET_LOCATION, FORWARD_VISION_SERVER_IP, FORWARD_VISION_SERVER_ID)
    pose, ret = client.getRemoteBufferContents(SENSORS_ANGULAR, SENSOR_SERVER_IP, SENSOR_SERVER_ID)
    orient = Unpack(LocationAndRotation, data)
    pool_position = Unpack(Angular, pose)

    # Converting from quaternion to euler pool coordinates
    quaternion = (
        pool_position.pos[QUAT_W],
        pool_position.pos[QUAT_X],
        pool_position.pos[QUAT_Y],
        pool_position.pos[QUAT_Z])
    euler = test_q2e(quaternion)
    roll = euler[xaxis]
    pitch = euler[yaxis]
    yaw = euler[zaxis]

    print("Correcting course...")
    orient_x = orient.xrot
    orient_y = orient.yrot
    orient_z = orient.zrot
    control_input.angular[xaxis].pos[POSITION] = 0
    control_input.angular[xaxis].pos[TIME] = 0
    control_input.angular[yaxis].pos[POSITION] = 0
    control_input.angular[yaxis].pos[TIME] = 0
    control_input.angular[zaxis].pos[POSITION] = yaw + orient_z
    control_input.angular[zaxis].pos[TIME] = 0

    return "SetDepth", cargo


def set_depth_transitions(cargo, previous):
    control_input.linear[zaxis].pos[POSITION] = DEPTH
    control_input.linear[zaxis].pos[TIME] = 0

    return "BuoyDeadReckon", cargo


def buoydr_transitions(cargo, previous):
    global color
    print("Dead Reckoning towards buoys...")
    time.sleep(.5)

    # step 1: broadcast goals to all other pis
    buoy_goals = Goals()
    buoy_goals.forwardVision = GOAL_FIND_RED_BUOY
    buoy_goals.downwardVision = GOAL_FIND_PATH
    buoy_goals.sonar = GOAL_NONE

    client.setLocalBufferContents(MASTER_GOALS, Pack(buoy_goals))

    # step 2: check if the robot is killed
    if previous == "PathOrientation" or previous == "BuoyDeadReckon":
        return "IsKilled", cargo

    # step 3: check if the robot sees the red buoy
    elif previous == "IsKilled":
        color = "red"
        print("can red be seen?")
        return "CheckBuoy", cargo

    # step 4: set velocity in m/s
    # setting angular Position
    control_input.angular[xaxis].pos[POSITION] = 0
    control_input.angular[xaxis].pos[TIME] = 0
    control_input.angular[yaxis].pos[POSITION] = 0
    control_input.angular[yaxis].pos[TIME] = 0
    control_input.angular[zaxis].pos[POSITION] = 0
    control_input.angular[zaxis].pos[TIME] = 0

    # setting linear velocity
    control_input.linear[xaxis].vel = XVEL
    control_input.linear[yaxis].vel = YVEL
    control_input.linear[zaxis].vel = ZVEL

    # setting the mode, with lin(z,y,x) and ang(z,y,x)
    control_input.mode = 39

    client.setLocalBufferContents(MASTER_CONTROL, Pack(control_input))

    return "BuoyDeadReckon", cargo


def checkbuoy_transitions(cargo, previous):
    global color
    print("Looking for the " + color + " buoy...")
    time.sleep(.5)
    data, active = client.getRemoteBufferContents(TARGET_LOCATION, FORWARD_VISION_SERVER_IP, FORWARD_VISION_SERVER_ID)
    pose, ret = client.getRemoteBufferContents(SENSORS_ANGULAR, SENSOR_SERVER_IP, SENSOR_SERVER_ID)
    see = Unpack(Location, data)
    pool_position = Unpack(Angular, pose)

    # step 1: broadcast goals to all other pis
    buoy_goals = Goals()
    buoy_goals.downwardVision = GOAL_FIND_PATH
    buoy_goals.sonar = GOAL_NONE
    if color == "yellow":
        buoy_goals.forwardVision = GOAL_FIND_YELLOW_BUOY
    elif color == "red":
        buoy_goals.forwardVision = GOAL_FIND_RED_BUOY
    client.setLocalBufferContents(MASTER_GOALS, Pack(buoy_goals))

    # step 2 Converting from quaternion to euler pool coordinates
    quaternion = (
        pool_position.pos[QUAT_W],
        pool_position.pos[QUAT_X],
        pool_position.pos[QUAT_Y],
        pool_position.pos[QUAT_Z])
    euler = test_q2e(quaternion)
    roll = euler[xaxis]
    pitch = euler[yaxis]
    yaw = euler[zaxis]

    # step 3 If the confidence level is met, correct course
    if color == "yellow":
        if see.confidence >= CONFIDENCE and see.loctype == YELLOW:
            buoy_found(control_input, see, yaw)
            return "SonarDeadReckon", cargo
    elif color == "red":
        if see.confidence >= CONFIDENCE and see.loctype == RED:
            buoy_found(control_input, see, yaw)
            return "BuoyVision", cargo
    print("no visual :(")
    return "BuoyDeadReckon", cargo


def buoy_found(buoy_control_input, see, yaw):
    print("We have a visual! Correcting course...")
    buoy_x = see.x
    buoy_y = see.y
    buoy_z = see.z
    buoy_control_input.angular[xaxis].pos[POSITION] = 0
    buoy_control_input.angular[xaxis].pos[TIME] = 0
    buoy_control_input.angular[yaxis].pos[POSITION] = 0
    buoy_control_input.angular[yaxis].pos[TIME] = 0
    buoy_control_input.angular[zaxis].pos[POSITION] = math.atan(buoy_y / buoy_x)
    buoy_control_input.angular[zaxis].pos[TIME] = 0
    mat_yaw = np.array([[math.cos(-yaw), -math.sin(-yaw)], [math.sin(-yaw), math.cos(-yaw)]])
    mat_pos = np.array([[buoy_x], [buoy_y]])
    mat_pos = np.dot(mat_yaw, mat_pos)
    # This velocity value is changeable, it is setting the total velocity when going towards the gate
    vel = VELOCITY
    buoy_control_input.linear[xaxis].vel = math.sqrt(pow(vel, 2) - math.pow(mat_pos[1, 0], 2))
    buoy_control_input.linear[yaxis].vel = mat_pos[1, 0]
    buoy_control_input.linear[zaxis].pos[POSITION] = 0
    buoy_control_input.linear[zaxis].pos[TIME] = 0


def buoyvision_transitions(cargo, previous):
    global color
    print("Moving to yellow Buoy...")
    time.sleep(.5)

    # step 1: Check if the robot is killed
    if previous == "CheckBuoy" or previous == "BuoyVision":
        return "IsKilled", cargo

    # step 2: Check/navigate to the yellow buoy
    elif previous == "IsKilled":
        color = "yellow"
        return "CheckBuoy", cargo

    return "BuoyVision", cargo


def sonarfinder_transitions(cargo, previous):
    print("in sonarfinder_transitions")

    data, active = client.getRemoteBufferContents(TARGET_LOCATION, SONAR_SERVER_IP, SONAR_SERVER_ID)
    see_path = Unpack(LocationAndRotation, data)

    if ord(see_path.confidence) >= CONFIDENCE:
        print("Detected Octogon Pinger!")
        return "SonarOrientation", cargo

    print("No pinger detected :(")
    return previous, cargo


def sonarorient_transitions(cargo, previous):
    print("Orienting robot towards pinger...")
    time.sleep(.5)
    data, active = client.getRemoteBufferContents(TARGET_LOCATION, FORWARD_VISION_SERVER_IP, FORWARD_VISION_SERVER_ID)
    pose, ret = client.getRemoteBufferContents(SENSORS_ANGULAR, SENSOR_SERVER_IP, SENSOR_SERVER_ID)
    orient = Unpack(LocationAndRotation, data)
    pool_position = Unpack(Angular, pose)

    if orient.confidence >= CONFIDENCE:
        return "EndOfRun", cargo

    # Converting from quaternion to euler pool coordinates
    quaternion = (
        pool_position.pos[QUAT_W],
        pool_position.pos[QUAT_X],
        pool_position.pos[QUAT_Y],
        pool_position.pos[QUAT_Z])
    euler = test_q2e(quaternion)
    roll = euler[xaxis]
    pitch = euler[yaxis]
    yaw = euler[zaxis]

    print("Correcting course...")
    orient_x = orient.xrot
    orient_y = orient.yrot
    orient_z = orient.zrot
    control_input.angular[xaxis].pos[POSITION] = 0
    control_input.angular[xaxis].pos[TIME] = 0
    control_input.angular[yaxis].pos[POSITION] = 0
    control_input.angular[yaxis].pos[TIME] = 0
    control_input.angular[zaxis].pos[POSITION] = yaw + orient_z
    control_input.angular[zaxis].pos[TIME] = 0

    return "OctoDeadReckon", cargo


def sonardr_transitions(cargo, previous):
    global color
    print("Dead Reckoning towards the octogon...")
    time.sleep(.5)

    # step 1: broadcast goals to all other pis
    buoy_goals = Goals()
    buoy_goals.forwardVision = GOAL_NONE
    buoy_goals.downwardVision = GOAL_NONE
    buoy_goals.sonar = GOAL_FIND_OCTOGON

    client.setLocalBufferContents(MASTER_GOALS, Pack(buoy_goals))

    # step 2: check if the robot is killed
    if previous == "SonarOrientation" or previous == "OctoDeadReckon":
        return "IsKilled", cargo

    if previous == "IsKilled":
        return "SonarFinder", cargo

    # step 3: set velocity in m/s
    # setting angular Position
    control_input.angular[xaxis].pos[POSITION] = 0
    control_input.angular[xaxis].pos[TIME] = 0
    control_input.angular[yaxis].pos[POSITION] = 0
    control_input.angular[yaxis].pos[TIME] = 0
    control_input.angular[zaxis].pos[POSITION] = 0
    control_input.angular[zaxis].pos[TIME] = 0

    # setting linear velocity
    control_input.linear[xaxis].vel = XVEL
    control_input.linear[yaxis].vel = YVEL
    control_input.linear[zaxis].vel = ZVEL

    # setting the mode, with lin(z,y,x) and ang(z,y,x)
    control_input.mode = 39

    client.setLocalBufferContents(MASTER_CONTROL, Pack(control_input))

    return "OctoDeadReckon", cargo


if __name__ == "__main__":
    print("Initializing Client")
    client = pydsm.Client(MASTER_SERVER_ID, 60, True)

    print("Creating Local Buffers")
    client.registerLocalBuffer(MASTER_CONTROL, sizeof(ControlInput), False)
    client.registerLocalBuffer(MASTER_GOALS, sizeof(Goals), False)
    client.registerLocalBuffer(MASTER_SENSOR_RESET, sizeof(SensorReset), False)
    time.sleep(1)
    control_input = ControlInput()
    goals = Goals()
    sensor_reset = SensorReset()
    client.setLocalBufferContents(MASTER_CONTROL, Pack(control_input))
    client.setLocalBufferContents(MASTER_GOALS, Pack(goals))
    client.setLocalBufferContents(MASTER_SENSOR_RESET, Pack(sensor_reset))
    # this is setting the initial sensor_reset.reset value to false
    sensor_reset = SensorReset()
    sensor_reset.reset = False

    print("Creating Remote Buffers")
    client.registerRemoteBuffer(SENSORS_LINEAR, SENSOR_SERVER_IP, SENSOR_SERVER_ID)
    client.registerRemoteBuffer(SENSORS_ANGULAR, SENSOR_SERVER_IP, SENSOR_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION, FORWARD_VISION_SERVER_IP, FORWARD_VISION_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION, DOWNWARD_VISION_SERVER_IP, DOWNWARD_VISION_SERVER_ID)
    client.registerRemoteBuffer(TARGET_LOCATION, SONAR_SERVER_IP, SONAR_SERVER_ID)
    client.registerRemoteBuffer(MOTOR_KILL, MOTOR_SERVER_IP, MOTOR_SERVER_ID)
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
    m.add_state("SetDepth", set_depth_transitions)
    m.add_state("BuoyDeadReckon", buoydr_transitions)
    m.add_state("CheckBuoy", checkbuoy_transitions)
    m.add_state("BuoyVision", buoyvision_transitions)
    m.add_state("SonarFinder", sonarfinder_transitions)
    m.add_state("SonarOrientation", sonarorient_transitions)
    m.add_state("OctoDeadReckon", sonardr_transitions)
    m.add_state("Error", None, end_state=1)
    m.add_state("EndOfRun", None, end_state=1)

    m.set_start("Kill")
    m.run("PLACEHOLDER")