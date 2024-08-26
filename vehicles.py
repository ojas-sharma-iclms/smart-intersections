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
import numpy as np
import csv

# CLASS DEFINITIONS

class RoadObject():

    roadPositions = []
    numOfObjects = 0

    def __init__(self, id, roadMap, roadSection, positionInSection):
        self.id = id
        self.roadSection = roadSection
        self.positionInSection = positionInSection
        self.position = (self.roadSection, self.positionInSection)
        self.roadMap = roadMap

class Vehicle(RoadObject):
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction):
        self.destination = destination
        self.velocity = velocity
        self.headway = 0
        self.acceleration = 0
        self.vmax = vmax
        self.amin = amin
        self.amax = amax
        self.hst = hst
        self.hgo = hgo
        self.turning = [] # whether the vehicle will need to turn at any intersections en-route
        self.vehicle_length = vehicle_length
        self.stepsPerSecond = stepsPerSecond
        self.human_reaction = human_reaction
        self.autonomous_reaction = autonomous_reaction
        self.path = []
        self.next = None
        RoadObject.__init__(self, id, roadMap, roadSection, positionInSection)

        self.headwayHistoryTau = [0 for x in range(round(self.stepsPerSecond * self.human_reaction))]
        self.headwayHistorySigma = [0 for x in range(round(self.stepsPerSecond * self.autonomous_reaction))]
        self.velocityHistoryTau = [0 for x in range(round(self.stepsPerSecond * self.human_reaction))]
        self.velocityHistorySigma = [0 for x in range(round(self.stepsPerSecond * self.autonomous_reaction))]
        self.accelerationHistoryTau = [0 for x in range(round(self.stepsPerSecond * self.human_reaction))]
        self.accelerationHistorySigma = [0 for x in range(round(self.stepsPerSecond * self.autonomous_reaction))]

        self.getOptimalRoute() # check if turning at any intersections en-route (code run only at beginning)
        pathIds = []
        for object in self.path:
            pathIds.append(object.split(".", 1)[0])
        counter = 0
        for id in pathIds:
            if (pathIds.count(id) == 3) and (id[0] == "I"):
                if self.path[np.where(np.array(pathIds)==id)[0][1]] not in self.turning: self.turning.append(self.path[np.where(np.array(pathIds)==id)[0][1]])
            if (pathIds.count(id) == 1) and (id[0] == "I"):
                self.turning.append(self.path[counter])
            counter += 1

    def getOptimalRoute(self):
        self.path = self.roadMap.getOptimalRoute(self.roadSection, self.destination)
        nextId = self.path[1].split(".", 1)[0]
        direction = self.path[1].split(".", 1)[1]
        self.next = self.roadMap.lookUp(nextId)
        return self.path

    def getTurns(self):
        pathIds = []
        for object in self.path:
            pathIds.append(object.split(".", 1)[0])
        for id in pathIds:
            if pathIds.count(id) == 3:
                if id not in self.turning: self.turning.append(id)
    
    def getPath(self):
        self.next_object = self.roadMap.getPath(self.id, self.roadSection, self.destination)
        return self.next_object
    
    def getDistanceToEndOfSection(self):
        road_id = self.roadMap.lookUp(self.roadSection.split(".", 1)[0])
        length = self.roadMap.roadMap[road_id].length
        return length - self.positionInSection
    
    def getHeadway(self):
        next_object = self.getPath()
        next_object_id = self.roadMap.lookUp(next_object)
        if ((next_object[0] == "R") or (next_object[0] == "I") or (next_object[0] == "S")):
            headway = self.getDistanceToEndOfSection()
            onPath = True
            counter = 1 # ignore the 0th term, as this is the section the car is in already
            while onPath:
                if self.path[counter].split(".", 1)[0] != next_object:
                    headway += self.roadMap.roadMap[self.roadMap.lookUp(self.path[counter].split(".", 1)[0])].length
                    counter += 1
                else:
                    onPath = False
        elif ((next_object[0] == "V")):
            # cars are complicated
            # need to check if the car is in the same section as the vehicle
            section = self.roadMap.roadMap[next_object_id].roadSection
            if self.roadSection == section:
                # same section
                headway = self.roadMap.roadMap[next_object_id].positionInSection - self.positionInSection
            else:
                # different sections
                headway = self.getDistanceToEndOfSection()
                onPath = True
                counter = 1 # ignore the 0th term, as this is the section the car is in already
                while onPath:
                    if self.path[counter] != section:
                        headway += self.roadMap.roadMap[self.roadMap.lookUp(self.path[counter].split(".", 1)[0])].length
                        counter += 1
                    else:
                        onPath = False
                headway += self.roadMap.roadMap[next_object_id].positionInSection
        headway = headway - self.vehicle_length
        return headway
        
    def updateDelays(self):
        self.headwayHistoryTau.insert(0, self.getHeadway())
        self.headwayHistorySigma.insert(0, self.getHeadway())
        self.headwayHistoryTau.pop()
        self.headwayHistorySigma.pop()

        self.velocityHistoryTau.insert(0, self.velocity)
        self.velocityHistorySigma.insert(0, self.velocity)
        self.velocityHistoryTau.pop()
        self.velocityHistorySigma.pop()

        self.accelerationHistoryTau.insert(0, self.acceleration)
        self.accelerationHistorySigma.insert(0, self.acceleration)
        self.accelerationHistoryTau.pop()
        self.accelerationHistorySigma.pop()
        
    def getHeadwayTau(self):
        return self.headwayHistoryTau[-1]
    
    def getHeadwaySigma(self):
        return self.headwayHistorySigma[-1]

    def getVelocityTau(self):
        return self.velocityHistoryTau[-1]

    def getVelocitySigma(self):
        return self.velocityHistorySigma[-1]
    
    def getAccelerationTau(self):
        return self.accelerationHistoryTau[-1]

    def getAccelerationSigma(self):
        return self.accelerationHistorySigma[-1]

class Human(Vehicle):
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction, ah, bh, yh):
        self.ah = ah
        self.bh = bh
        self.yh = yh
        self.type = "human"
        Vehicle.__init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction)
    
    def velocityError(self):
        headway = self.getHeadwayTau()
        if (headway <= self.hst):
            calc = 0
        elif (headway >= self.hgo):
            calc = self.vmax
        elif ((headway < self.hgo) and (headway > self.hst)):
            calc = ((self.vmax / 2) * (1 - np.cos(np.pi * ((headway - self.hst) / (self.hgo - self.hst)))))
        diff = calc - self.getVelocityTau()
        return (self.ah * diff)
    
    def velocityDelta(self):
        next_object = self.getPath()
        next_id = self.roadMap.lookUp(next_object)
        if (next_object[0] == "S"):
            next_velocity = self.vmax
        elif (next_object[0] == "V"):
            next_velocity = self.roadMap.roadMap[next_id].velocity
        elif (next_object[0] == "I"):
            next_velocity = 0
        elif (next_object[0] == "R"):
            print("HOW IS THE NEXT STOPPABLE OBJECT A ROAD??!!?")
            next_velocity = 0
        calc = next_velocity - self.velocity
        return (self.bh * calc)
    
    def accelerationDelta(self):
        pass

class Autonomous(Vehicle):
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction, alpha, beta, gamma):
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.type = "autonomous"
        self.vehiclesSeen = []
        self.vehiclesOnPath = []
        Vehicle.__init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction)
    
    def velocityError(self):
        headway = self.getHeadwaySigma()
        if (headway <= self.hst):
            calc = 0
        elif (headway >= self.hgo):
            calc = self.vmax
        elif ((headway < self.hgo) and (headway > self.hst)):
            calc = ((self.vmax / 2) * (1 - np.cos(np.pi * ((headway - self.hst) / (self.hgo - self.hst)))))
        diff = calc - self.getVelocitySigma()
        return (self.alpha * diff)
    
    def velocityDelta(self):
        pass

    def accelerationDelta(self):
        pass

    def cascade(self):
        if (self.roadSection[0] == "I"):
            self.vehiclesSeen = []
        else:
            nextIntersectionFound = False
            tempPath = self.path
            cascadePath = []
            tempCount = 0
            while not nextIntersectionFound:
                if tempPath[tempCount][0] != "I":
                    cascadePath.append(tempPath[tempCount])
                    if (tempCount == len(tempPath) - 1):
                        nextIntersectionFound = True
                    tempCount += 1
                else:
                    nextIntersectionFound = True
            found = False
            counter = 0
            while not found:
                vehicle = self.roadMap.roadMap[self.roadMap.lookUp(cascadePath[0].split(".", 1)[0])].stack[int(cascadePath[0].split(".", 1)[1])][counter]
                if (vehicle == self.id):
                    found = True
                else:
                    counter += 1
                    found = False
            if counter != 0:
                for i in range(counter-1, -1, -1):
                    self.vehiclesOnPath.append(self.roadMap.roadMap[self.roadMap.lookUp(cascadePath[0].split(".", 1)[0])].stack[int(cascadePath[0].split(".", 1)[1])][i])

            cascadePath.pop(0)

            for roadSection in cascadePath:
                for vehicleOnPath in reversed(self.roadMap.roadMap[self.roadMap.lookUp(roadSection.split(".", 1)[0])].stack[int(roadSection.split(".", 1)[1])]):
                    self.vehiclesOnPath.append(vehicleOnPath)
        
            autonomousFound = False
            anotherCounter = 0
            for element in self.vehiclesOnPath:
                if self.roadMap.roadMap[self.roadMap.lookUp(element)].type == "autonomous":
                    autonomousFound = True
                else:
                    anotherCounter += 1
            if not autonomousFound:
                anotherCounter -= 1

            self.vehiclesSeen = self.vehiclesOnPath[0:anotherCounter+1]
            return self.vehiclesSeen