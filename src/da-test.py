import sys
sys.path.insert(0, '../DistributedSharedMemory/build')
sys.path.insert(0, '../PythonSharedBuffers/src')
from Constants import *
import pydsm
import os
import time
import pprint
from ctypes import *
from Sensor import *
from Master import *
from Navigation import *
from Vision import *
from Serialization import *


def setKill(isKilled):
    motor.isKilled = isKilled
    motorClient.setLocalBufferContents(MOTOR_KILL, Pack(motor))

def set(type, file):
    if (type == 'fv'):
        getVarFromFile('TestingVars/FV/' + file)
        forwardVisionClient.setLocalBufferContents(TARGET_LOCATION_AND_ROTATION, Pack(data.data))
    elif (type == 'dv'):
        getVarFromFile('TestingVars/DV/' + file)
        downwardVisionClient.setLocalBufferContents(TARGET_LOCATION_AND_ROTATION, Pack(data.data))
    elif (type == 'sonar'):
        getVarFromFile('TestingVars/Sonar/' + file)
        sonarClient.setLocalBufferContents(TARGET_LOCATION_AND_ROTATION, Pack(data.data))
    else:
        print ('Unknown data type')
    pp(data.data)

def getVarFromFile(filename):
    import imp
    f = open(filename)
    print ('reading file')
    global data
    data = imp.load_source('data', '', f)
    f.close()

def pp(s):
    print ('printing data:')
    for field_name, field_type in s._fields_:
        print(field_name, getattr(s, field_name))


if __name__ == "__main__":
    print("Initializing Client")
    sensorClient = pydsm.Client(SENSOR_SERVER_ID, 71, True)
    motorClient = pydsm.Client(MOTOR_SERVER_ID, 72, True)
    forwardVisionClient = pydsm.Client(FORWARD_VISION_SERVER_ID, 73, True)
    downwardVisionClient = pydsm.Client(DOWNWARD_VISION_SERVER_ID, 74, True)
    sonarClient = pydsm.Client(SONAR_SERVER_ID, 75, True)

    print("Creating Local Buffers")
    sensorClient.registerLocalBuffer(SENSORS_LINEAR, sizeof(Linear), False)
    sensorClient.registerLocalBuffer(SENSORS_ANGULAR, sizeof(Angular), False)
    forwardVisionClient.registerLocalBuffer(TARGET_LOCATION, sizeof(LocationArray), False)
    downwardVisionClient.registerLocalBuffer(TARGET_LOCATION_AND_ROTATION, sizeof(LocationArray), False)
    sonarClient.registerLocalBuffer(TARGET_LOCATION, sizeof(LocationArray), False)
    motorClient.registerLocalBuffer(MOTOR_KILL, sizeof(Kill), False)
    time.sleep(1)
    linear = Linear()
    angular = Angular()
    fvLoc = LocationArray()
    dvLoc = LocationArray()
    sonarLoc = LocationArray()
    motor = Kill()
    sensorClient.setLocalBufferContents(SENSORS_LINEAR, Pack(linear))
    sensorClient.setLocalBufferContents(SENSORS_ANGULAR, Pack(angular))
    forwardVisionClient.setLocalBufferContents(TARGET_LOCATION_AND_ROTATION, Pack(fvLoc))
    downwardVisionClient.setLocalBufferContents(TARGET_LOCATION_AND_ROTATION, Pack(dvLoc))
    sonarClient.setLocalBufferContents(TARGET_LOCATION, Pack(sonarLoc))
    setKill(True)

    print("Creating Remote Buffers")
    sensorClient.registerRemoteBuffer(MASTER_CONTROL, MASTER_SERVER_IP, MASTER_SERVER_ID)
    sensorClient.registerRemoteBuffer(MASTER_GOALS,  MASTER_SERVER_IP, MASTER_SERVER_ID)
    sensorClient.registerRemoteBuffer(MASTER_SENSOR_RESET,  MASTER_SERVER_IP, MASTER_SERVER_ID)
    time.sleep(1)
    sensorreset = SensorReset()
    sensorreset.reset = False

    try:
        print("Commands: kill; unkill; set type fileName; exit")
        while (1):
            tokens = input("> ").split()
            if (tokens[0] == "exit"):
                print("exiting")
                break
            elif (tokens[0] == "kill"):
                print("Killing the bot")
                setKill(True)
            elif (tokens[0] == "unkill"):
                print("Unkilling the bot")
                setKill(False)
            elif (tokens[0] == "set" and len(tokens) == 3):
                print("Updating values...")
                set(tokens[1], tokens[2])
            else:
                print("unknown")
    except KeyboardInterrupt:
        print("")
        print("exiting")