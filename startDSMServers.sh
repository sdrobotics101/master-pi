#!/bin/sh
DistributedSharedMemory/DSMServer -f -s 42 > Master.log 2>&1 &
DistributedSharedMemory/DSMServer -f -s 43 > Sensor.log 2>&1 &
DistributedSharedMemory/DSMServer -f -s 44 > FV.log 2>&1 &
DistributedSharedMemory/DSMServer -f -s 45 > DV.log 2>&1 &
DistributedSharedMemory/DSMServer -f -s 46 > Sonar.log 2>&1 &
DistributedSharedMemory/DSMServer -f -s 47 > Motor.log 2>&1 &
