''' Smart Intersections - Simulation of human-driven and autonomous vehicles in a city environment.
    Copyright (C) 2024 Ojas Sharma

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

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

print("\nSmart Intersections - Simulation of human-driven and autonomous vehicles in a city environment.\nCopyright (C) 2024 Ojas Sharma.\nThis program comes with ABSOLUTELY NO WARRANTY.\nThis is free software, and you are welcome to redistribute it under certain conditions.\n")

map = RoadNetwork('map1')
map.plotMapGraph()

test = Autonomous("V1", map, "S1.1", 0.66, "S3.0", 24, 30, -6, 2, 5, 50, 10, 0.05, 1, 0.2, 0.4, 0.1)
map.roadMap.append(test)

print(test.getOptimalRoute())
print(test.next)
print(map.roadMap[test.next].id)

map.updateStacks() # must be called every timestep

print(map.roadMap[map.lookUp("S3")].stack)