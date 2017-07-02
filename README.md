# the-brain-thing
> The only properly named repo in this entire project!

this code shall control everything that the robot does. it is on the one true
raspberry pi and shall control all other raspberry pis.

For creating testing files:

Step 1: Copy/paste this code at the top of your file

>import sys

>sys.path.insert(0, '../PythonSharedBuffers/src')

>from Constants import *

>import time

>from ctypes import *

>from Sensor import *

>from Master import *

>from Navigation import *

>from Vision import *

>from Serialization import *

Step 2: Edit the data that is being read in by the tester:

ex:

>data = LocationAndRotation()

>data.x = 1

Step 3: put it in the appropriate TestingVars folder (DV, FV, Sonar, etc)