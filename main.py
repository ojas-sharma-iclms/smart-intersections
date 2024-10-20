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
yh = 0.5
alpha = 1.2
beta = 0.2
gamma = 0.5
vmax = 30
hst = 5
hgo = 100
vehicle_length = 5
amin = -6
amax = 3
stepsPerSecond = 50
running = True

# CODE

print("\nSmart Intersections - Simulation of human-driven and autonomous vehicles in a city environment.\nCopyright (C) 2024 Ojas Sharma.\nThis program comes with ABSOLUTELY NO WARRANTY.\nThis is free software, and you are welcome to redistribute it under certain conditions.\n")

map = RoadNetwork('map1')
map.plotMapGraph()

# make sure each vehicle has a different id!!

test = Human("V1", map, "R1.1", 300, "S3.0", 0, 30, -6, 2, 5, 50, 10, 50, 1, 0.2, 0.4, 0.1, 0.5)
map.roadMap.append(test)

test2 = Human("V2", map, "R1.1", 200, "S2.0", 30, 30, -6, 2, 5, 50, 10, 50, 1, 0.2, 0.4, 0.1, 0.5)
map.roadMap.append(test2)

testAutonomous = Autonomous("V3", map, "R1.1", 40, "S3.0", 0, 30, -6, 2, 5, 50, 10, 50, 1, 0.2, 0.4, 2, 2)
map.roadMap.append(testAutonomous)

map.finaliseRoadMap()

# ----- MAIN LOOP -----

seconds = 0
map.updateStacks() # first initialise the map, so getHeadway() can be called

for object in map.roadMap: # set the headway history to current headway, AFTER initialising the map
    if ((object.type == "human") or (object.type == "autonomous")):
        object.updateHeadwayHistory(object.getHeadway())

while running:
    stepNum = 0
    for stepNum in range(stepsPerSecond):

        for object in map.roadMap:
            if (object.type == "autonomous"):
                object.cascade()
        for object in map.roadMap:
            if ((object.type == "human") or (object.type == "autonomous")):
                object.getHeadway()
        for object in map.roadMap:
            if ((object.type == "human") or (object.type == "autonomous")):
                object.optimalVelocity()
        for object in map.roadMap:
            if ((object.type == "human") or (object.type == "autonomous")):
                object.updateVelocity()
        for object in map.roadMap:
            if ((object.type == "human") or (object.type == "autonomous")):
                object.updatePositions()
        for object in map.initialRoadMap:
            if ((object.type == "human") or (object.type == "autonomous")):
                object.saveData()

        map.updateStacks() # update the map for the NEXT timestep

    usr = input()

    if (usr == 'end'):
        running = False
    if ((usr == 'show') or (map.mapEmpty)):
        running = False
        seconds += 1

        plt.close('all')
        fig, ax = plt.subplots(
        ncols=1, nrows=3, figsize=(10, 5.4), layout="constrained", sharex=True
    )
        velocityData = []
        accelData = []
        headwayData = []

        for object in map.initialRoadMap:
            if ((object.type == "human") or (object.type == "autonomous")):
                velocityData.append(object.velocityData)
                accelData.append(object.accelerationData)
                headwayData.append(object.headwayData)

        stepsPassed = seconds * stepsPerSecond
        for VELreading in velocityData:
            ax[0].plot(range(stepsPassed), VELreading)
        for ACCreading in accelData:
            ax[1].plot(range(stepsPassed), ACCreading)
        for HEAreading in headwayData:
            ax[2].plot(range(stepsPassed), HEAreading)
        ax[0].set_ylabel("Velocity")
        ax[1].set_ylabel("Acceleration")
        ax[2].set_ylabel("Headway")
        plt.xlabel("Timesteps")
        plt.show()
    
    seconds += 1