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
alpha = 0.2
beta = 0.2
gamma = 0.5
vmax = 30
hst = 5
hgo = 100
acceleration_smoothing = 0.05
vehicle_length = 5
amin = -6
amax = 3
stepsPerSecond = 50
saveInterval = 0.5 # in seconds
running = True

# TIME SPACE DIAGRAM ANALYSIS

start = "R1.1"
stop = "S2.0"

# CODE

print("\nSmart Intersections - Simulation of human-driven and autonomous vehicles in a city environment.\nCopyright (C) 2024 Ojas Sharma.\nThis program comes with ABSOLUTELY NO WARRANTY.\nThis is free software, and you are welcome to redistribute it under certain conditions.\n")

mapName = "map1"
map = RoadNetwork(mapName)
map.plotMapGraph()

# make sure each vehicle has a different id!!

test5  = Human("V6", map, "R1.1", 450, "S2.0", 10, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, humanReaction, autonomousReaction, ah, bh, yh, acceleration_smoothing)
map.roadMap.append(test5)

test4  = Human("V5", map, "R1.1", 400, "S2.0", 30, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, humanReaction, autonomousReaction, ah, bh, yh, acceleration_smoothing)
map.roadMap.append(test4)

test = Human("V1", map, "R1.1", 300, "S3.0", 30, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, humanReaction, autonomousReaction, ah, bh, yh, acceleration_smoothing)
map.roadMap.append(test)

test2 = Human("V2", map, "R1.1", 200, "S2.0", 30, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, humanReaction, autonomousReaction, ah, bh, yh, acceleration_smoothing)
map.roadMap.append(test2)

testAutonomous = Autonomous("V3", map, "R1.1", 50, "S3.0", 30, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, humanReaction, autonomousReaction, alpha, beta, gamma, acceleration_smoothing)
map.roadMap.append(testAutonomous)

test3 = Human("V4", map, "R1.1", 10, "S2.0", 0, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, humanReaction, autonomousReaction, ah, bh, yh, acceleration_smoothing)
map.roadMap.append(test3)

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
        velocityDataA = []
        accelDataA = []
        headwayDataA = []

        for object in map.initialRoadMap:
            if ((object.type == "human")):
                velocityData.append(object.velocityData)
                accelData.append(object.accelerationData)
                headwayData.append(object.headwayData)
            if ((object.type == "autonomous")):
                velocityDataA.append(object.velocityData)
                accelDataA.append(object.accelerationData)
                headwayDataA.append(object.headwayData)

        stepsPassed = seconds * stepsPerSecond
        for VELreading in velocityData:
            ax[0].plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), VELreading)
        for ACCreading in accelData:
            ax[1].plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), ACCreading)
        for HEAreading in headwayData:
            ax[2].plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), HEAreading)
        for VELreading in velocityDataA:
            ax[0].plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), VELreading, linestyle='dashed')
        for ACCreading in accelDataA:
            ax[1].plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), ACCreading, linestyle='dashed')
        for HEAreading in headwayDataA:
            ax[2].plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), HEAreading, linestyle='dashed')
        ax[0].set_ylabel("Velocity (m/s)")
        ax[1].set_ylabel("Acceleration (m/s^2)")
        ax[2].set_ylabel("Headway (metres)")
        plt.xlabel("Time (seconds)")
        plt.show()
    
    seconds += 1

# TIME-SPACE DIAGRAM

absPosData = []
absPosDataA = []

plt.close('all')
routeToAnalyse = start + stop

for object in map.initialRoadMap:
    if (object.type == "human"):
        if (object.route == routeToAnalyse):
            absPosData.append(object.absolutePositionData)
    if (object.type == "autonomous"):
        if (object.route == routeToAnalyse):
            absPosDataA.append(object.absolutePositionData)

for ABSpos in absPosData:
    plt.plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), ABSpos)
for ABSpos in absPosDataA:
    plt.plot(np.arange(stepsPassed/stepsPerSecond,step=1/stepsPerSecond), ABSpos, linestyle='dashed')
plt.xlabel("Time (seconds)")
plt.ylabel(f"Position Along Route from {start} to {stop} (metres)")
plt.show()

# WRITE DATA TO CSV

NumOfVehicles = len(velocityData) + len(velocityDataA)
velocityHeaders = [f"Velocity{x+1}" for x in range(len(velocityData))] + [f"VelocityAV{x+1}" for x in range(len(velocityDataA))]
headwayHeaders = [f"Headway{x+1}" for x in range(len(headwayData))] + [f"HeadwayAV{x+1}" for x in range(len(headwayDataA))]
accelHeaders = [f"Acceleration{x+1}" for x in range(len(accelData))] + [f"AccelerationAV{x+1}" for x in range(len(accelDataA))]
absPosHeaders = [f"AbsPos{x+1}" for x in range(len(absPosData))] + [f"AbsPosAV{x+1}" for x in range(len(absPosDataA))]
allVelocities = list(zip(*(velocityData + velocityDataA)))
allHeadways = list(zip(*(headwayData + headwayDataA)))
allAccels = list(zip(*(accelData + accelDataA)))
allAbsPos = list(zip(*(absPosData + absPosDataA)))

with open(f"data/{mapName}.csv", "w", newline="") as cavData:
    dataWriter = csv.writer(cavData)

    headers = (["Time"] + velocityHeaders + accelHeaders + headwayHeaders)
    dataWriter.writerow(headers)

    for x in np.arange(stepsPassed, step=int(stepsPerSecond*saveInterval)):
        row = ([x / stepsPerSecond] + list(allVelocities[x]) + list(allAccels[x]) + list(allHeadways[x]))
        dataWriter.writerow(row)
    cavData.close()

with open(f"data/{mapName}_{routeToAnalyse}.csv", "w", newline="") as cavData:
    dataWriter = csv.writer(cavData)

    headers = (["Time"] + absPosHeaders)
    dataWriter.writerow(headers)

    for x in np.arange(stepsPassed, step=int(stepsPerSecond*saveInterval)):
        row = ([x / stepsPerSecond] + list(allAbsPos[x]))
        dataWriter.writerow(row)