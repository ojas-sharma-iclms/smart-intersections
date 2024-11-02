''' Smart Intersections - Simulation of human-driven and autonomous vehicles in a city environment.
    Copyright (C) 2024 Ojas Sharma

    Special thanks to Kai Legum for extensive help in the early logical stages of the implementation
    of the governing equations.

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
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction, acceleration_smoothing):
        self.destination = destination
        self.velocity = velocity
        self.headway = 0
        self.acceleration = 0
        self.vmax = vmax
        self.amin = amin
        self.amax = amax
        self.hst = hst
        self.hgo = hgo
        self.c = acceleration_smoothing
        self.onMap = True # is the vehicle still on the map?
        self.turning = [] # whether the vehicle will need to turn at any intersections en-route
        self.vehicle_length = vehicle_length
        self.stepsPerSecond = stepsPerSecond
        self.human_reaction = human_reaction
        self.autonomous_reaction = autonomous_reaction
        self.path = []
        self.absolutePosition = positionInSection
        self.next = None

        self.velocityData = []
        self.accelerationData = []
        self.headwayData = []
        self.roadSectionData = []
        self.absolutePositionData = []

        RoadObject.__init__(self, id, roadMap, roadSection, positionInSection)

        self.route = roadSection + destination

        self.headwayHistoryTau = [0 for x in range(round(self.stepsPerSecond * self.human_reaction))]
        self.headwayHistorySigma = [0 for x in range(round(self.stepsPerSecond * self.autonomous_reaction))]
        self.velocityHistoryTau = [self.velocity for x in range(round(self.stepsPerSecond * self.human_reaction))]
        self.velocityHistorySigma = [self.velocity for x in range(round(self.stepsPerSecond * self.autonomous_reaction))]
        self.accelerationHistoryTau = [self.acceleration for x in range(round(self.stepsPerSecond * self.human_reaction))]
        self.accelerationHistorySigma = [self.acceleration for x in range(round(self.stepsPerSecond * self.autonomous_reaction))]

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
    
    def updateHeadwayHistory(self, headway):
        self.headwayHistoryTau = [headway for x in range(round(self.stepsPerSecond * self.human_reaction))]
        self.headwayHistorySigma = [headway for x in range(round(self.stepsPerSecond * self.autonomous_reaction))]
    
    def saveData(self):
        if (self.onMap == True):
            self.roadSectionData.append(self.roadSection)
            self.headwayData.append(self.headway)
            self.velocityData.append(self.velocity)
            self.accelerationData.append(self.acceleration)
            self.absolutePositionData.append(self.absolutePosition)
        else:
            self.roadSectionData.append("NOT ON MAP")
            self.headwayData.append(np.nan)
            self.velocityData.append(np.nan)
            self.accelerationData.append(np.nan)
            self.absolutePositionData.append(np.nan)

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
        self.headway = headway
        # update delays
        self.headwayHistoryTau.insert(0, self.headway)
        self.headwayHistorySigma.insert(0, self.headway)
        self.headwayHistoryTau.pop()
        self.headwayHistorySigma.pop()
        return headway
        
    def updateDelays(self): # don't run this!!
        headway = self.getHeadway()
        self.headwayHistoryTau.insert(0, headway)
        self.headwayHistorySigma.insert(0, headway)
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
    
    def optimalVelocity(self):
        v_error = self.velocityError()
        v_delta = self.velocityDelta()
        a_delta = self.accelerationDelta()
        a = (v_error + v_delta + a_delta)

        if (a <= self.amin - self.c): # smoothing function for acceleration
            self.acceleration = self.amin
        elif a < self.amin + self.c:
            self.acceleration = a + ((self.amin - a + self.c) ** 2 / (4 * self.c))
        elif a <= self.amax - self.c:
            self.acceleration = a
        elif a < self.amax + self.c:
            self.acceleration = a - ((self.amax - a - self.c) ** 2 / (4 * self.c))
        else:
            self.acceleration = self.amax

        # update delays
        self.accelerationHistoryTau.insert(0, self.acceleration)
        self.accelerationHistorySigma.insert(0, self.acceleration)
        self.accelerationHistoryTau.pop()
        self.accelerationHistorySigma.pop()
    
    def updateVelocity(self):
        self.velocity = self.velocity + (self.acceleration / self.stepsPerSecond)
        # update delays
        self.velocityHistoryTau.insert(0, self.velocity)
        self.velocityHistorySigma.insert(0, self.velocity)
        self.velocityHistoryTau.pop()
        self.velocityHistorySigma.pop()

    def updatePositions(self):
        velocityStep = self.velocity / self.stepsPerSecond
        self.absolutePosition += velocityStep
        section = self.roadSection
        next_section = self.path[1] # must be run AFTER self.getHeadway()

        section_length = self.roadMap.roadMap[self.roadMap.lookUp(section.split(".", 1)[0])].length
        if (self.positionInSection + velocityStep) < section_length:
            # remains in current section
            self.positionInSection = self.positionInSection + velocityStep
        else:
            # joins next section
            diff = self.positionInSection + velocityStep - section_length
            # leave current section
            self.roadMap.roadMap[self.roadMap.lookUp(section.split(".", 1)[0])].leaveSection(self.id, section.split(".", 1)[1])
            # join next section
            if next_section[0] == "S":
                self.roadMap.roadMap[self.roadMap.lookUp(next_section.split(".", 1)[0])].leaveSection(self.id, next_section.split(".", 1)[1]) # leave the entry/exit too!
                self.roadMap.removeFromMap(self.id) # remove from map at end of journey
                self.onMap = False
            else:
                self.roadMap.roadMap[self.roadMap.lookUp(next_section.split(".", 1)[0])].joinSection(self.id, next_section.split(".", 1)[1])
                self.roadSection = next_section
                self.positionInSection = diff
            print(f"Vehicle {self.id} has left section {section} and joined section {next_section}")

class Human(Vehicle):
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction, ah, bh, yh, acceleration_smoothing):
        self.ah = ah
        self.bh = bh
        self.yh = yh
        self.type = "human"
        Vehicle.__init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction, acceleration_smoothing)
    
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
            calc = 0
        elif (next_object[0] == "V"):
            next_velocity = self.roadMap.roadMap[next_id].getVelocityTau()
            calc = next_velocity - self.getVelocityTau()
        elif (next_object[0] == "I"):
            calc = 0
        elif (next_object[0] == "R"):
            print("HOW IS THE NEXT STOPPABLE OBJECT A ROAD??!!?")
            calc = 0
        return (self.bh * calc)
    
    def accelerationDelta(self):
        next_object = self.getPath()
        next_id = self.roadMap.lookUp(next_object)
        if (next_object[0] == "S"):
            calc = 0
        elif (next_object[0] == "V"):
            next_acceleration = self.roadMap.roadMap[next_id].getAccelerationTau()
            calc = next_acceleration - self.getAccelerationTau()
            if ((self.roadMap.roadMap[next_id].acceleration <= 0) and (self.getAccelerationTau() > 0)): # sign function g(stuff)
                calc = calc * -1
        elif (next_object[0] == "I"):
            calc = 0
        elif (next_object[0] == "R"):
            print("HOW IS THE NEXT STOPPABLE OBJECT A ROAD??!!?")
            calc = 0
        return (self.yh * calc)

class Autonomous(Vehicle):
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction, alpha, beta, gamma, acceleration_smoothing):
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.k = 2 # must be greater than 1; the higher the number, the less that vehicles further away matter
        self.type = "autonomous"
        self.vehiclesSeen = []
        self.vehiclesOnPath = []
        Vehicle.__init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, stepsPerSecond, human_reaction, autonomous_reaction, acceleration_smoothing)
    
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
    
    def velocityDeltaSimple(self):
        next_object = self.getPath()
        next_id = self.roadMap.lookUp(next_object)
        if (next_object[0] == "S"):
            calc = 0
        elif (next_object[0] == "V"):
            next_velocity = self.roadMap.roadMap[next_id].getVelocitySigma()
            calc = next_velocity - self.getVelocitySigma()
        elif (next_object[0] == "I"):
            calc = 0
        elif (next_object[0] == "R"):
            print("HOW IS THE NEXT STOPPABLE OBJECT A ROAD??!!?")
            calc = 0
        return (self.beta * calc)
    
    def accelerationDeltaSimple(self):
        next_object = self.getPath()
        next_id = self.roadMap.lookUp(next_object)
        if (next_object[0] == "S"):
            calc = 0
        elif (next_object[0] == "V"):
            next_acceleration = self.roadMap.roadMap[next_id].getAccelerationSigma()
            calc = next_acceleration - self.getAccelerationSigma()
            if ((self.roadMap.roadMap[next_id].acceleration <= 0) and (self.getAccelerationSigma() > 0)): # sign function g(stuff)
                calc = calc * -1
        elif (next_object[0] == "I"):
            calc = 0
        elif (next_object[0] == "R"):
            print("HOW IS THE NEXT STOPPABLE OBJECT A ROAD??!!?")
            calc = 0
        return (self.gamma * calc)
    
    def velocityDelta(self):
        calc = 0
        tempCount = 1
        if len(self.vehiclesSeen) == 0:
            diff = self.velocityDeltaSimple()
        else:
            for vehicle in self.vehiclesSeen:
                next_id = self.roadMap.lookUp(vehicle)
                type = self.roadMap.roadMap[next_id].type
                if (type == "entry_exit"):
                    calc += 0
                elif ((type == "autonomous") or (type == "human")):
                    temp = self.roadMap.roadMap[next_id].getVelocitySigma() - self.getVelocitySigma()
                    if tempCount == len(self.vehiclesSeen):
                        multiplier = self.beta * (1 / ((self.k)**(tempCount-1)))
                    else:
                        multiplier = self.beta * ((self.k-1) / ((self.k)**(tempCount)))
                    calc += (multiplier * temp)
                elif (type == "intersection"):
                    calc += 0
                elif (type == "road"):
                    print("HOW IS THE NEXT STOPPABLE OBJECT A ROAD??!!?")
                    calc += 0
                tempCount += 1
            if calc == 0:
                calc = self.getVelocitySigma()
            #diff = calc - self.getVelocitySigma()
            diff = calc
        return diff

    def accelerationDelta(self):
        calc = 0
        tempCount = 1
        if len(self.vehiclesSeen) == 0:
            diff = self.accelerationDeltaSimple()
        else:
            for vehicle in self.vehiclesSeen:
                next_id = self.roadMap.lookUp(vehicle)
                type = self.roadMap.roadMap[next_id].type
                if (type == "entry_exit"):
                    calc += 0
                elif ((type == "autonomous") or (type == "human")):
                    temp = self.roadMap.roadMap[next_id].getAccelerationSigma() - self.getAccelerationSigma()
                    if tempCount == len(self.vehiclesSeen):
                        multiplier = self.gamma * (1 / ((self.k)**(tempCount-1)))
                    else:
                        multiplier = self.gamma * ((self.k-1) / ((self.k)**(tempCount)))
                    if ((self.roadMap.roadMap[next_id].acceleration <= 0) and (self.acceleration > 0)): # acceleration delta sign g(stuff)
                        temp = temp * -1
                    calc += (multiplier * temp)
                elif (type == "intersection"):
                    calc += 0
                elif (type == "road"):
                    print("HOW IS THE NEXT STOPPABLE OBJECT A ROAD??!!?")
                    calc += 0
                tempCount += 1
            if calc == 0:
                calc = self.getAccelerationSigma()
            #diff = ((calc - self.getAccelerationSigma()))
            diff = calc
        return diff

    def cascade(self):
        if (self.roadSection[0] == "I"):
            self.vehiclesSeen = []
            self.vehiclesOnPath = []
        else:
            nextIntersectionFound = False
            tempPath = self.path
            cascadePath = []
            self.vehiclesOnPath = []
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
            if self.id in self.vehiclesSeen:
                self.vehiclesSeen.remove(self.id) # fix some dodginess with more dodginess??
            return self.vehiclesSeen