import sys
sys.path.insert(0, '../PythonSharedBuffers/src')
from Constants import *
import time
from ctypes import *
from Sensor import *
from Master import *
from Navigation import *
from Vision import *
from Serialization import *
import pprint

def getVarFromFile(filename):
  import imp
  f = open(filename)
  global data
  data = imp.load_source('data', '', f)
  f.close()

def pp(s):
  for field_name, field_type in s._fields_:
    print(field_name, getattr(s, field_name))


if __name__ == "__main__":
  getVarFromFile('test1')
  pp(data.data)
  getVarFromFile('test2')
  pp(data.data)


