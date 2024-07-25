# LIBRARIES

import math
import networkx as nx
import matplotlib.pyplot as plt
import pygame, sys
from statistics import stdev
from random import randint
import csv

# MODULES

from routing import *
from vehicles import *
from pathing import *

# PARAMETERS

humanReaction = 1
autonomousReaction = 0.2
ah = 0.2
bh = 0.2
alpha = 1.2
beta = 0.2
vmax = 30
hst = 5
hgo = 100
vehicle_length = 5
c = 0.05
amin = -6
amax = 3
stepsPerSecond = 50

# CODE

map = RoadNetwork('map1')
map.plotMapGraph()

test = Autonomous("V1", map, "S1.1", 0.66, "S3.0", 24, 30, -6, 2, 5, 50, 10, 0.05, 1, 0.2, 0.4, 0.1)
map.roadMap.append(test)

print(test.getOptimalRoute())
print(test.next)
print(map.roadMap[test.next].id)

map.updateStacks() # must be called every timestep

print(map.roadMap[map.lookUp("S3")].stack)