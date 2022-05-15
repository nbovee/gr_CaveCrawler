from spike import PrimeHub, DistanceSensor, Motor, MotorPair
from math import radians, cos, sin, pi, copysign
from utime import sleep as wait_for_seconds

import uos as os

import gc
import array

print(os.uname())
print(os.getcwd())
print(os.listdir())
print(help("modules"))

# with open('/hub_runtime.mpy', 'r') as f:
    # print(f.read())