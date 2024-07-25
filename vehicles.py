# LIBRARIES

import math
import networkx as nx
import matplotlib.pyplot as plt
import pygame, sys
from statistics import stdev
from random import randint
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
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, c, human_reaction, autonomous_reaction):
        self.destination = destination
        self.velocity = velocity
        self.vmax = vmax
        self.amin = amin
        self.amax = amax
        self.hst = hst
        self.hgo = hgo
        self.vehicle_length = vehicle_length
        self.c = c
        self.human_reaction = human_reaction
        self.autonomous_reaction = autonomous_reaction
        self.path = []
        self.next = None
        RoadObject.__init__(self, id, roadMap, roadSection, positionInSection)

    def getOptimalRoute(self):
        self.path = self.roadMap.getOptimalRoute(self.roadSection, self.destination)
        nextId = self.path[1].split(".", 1)[0]
        direction = self.path[1].split(".", 1)[1]
        self.next = self.roadMap.lookUp(nextId)
        return self.path

class Human(Vehicle):
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, c, human_reaction, autonomous_reaction, ah, bh):
        self.ah = ah
        self.bh = bh
        self.type = "human"
        Vehicle.__init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, c, human_reaction, autonomous_reaction)

class Autonomous(Vehicle):
    def __init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, c, human_reaction, autonomous_reaction, alpha, beta):
        self.alpha = alpha
        self.beta = beta
        self.type = "autonomous"
        Vehicle.__init__(self, id, roadMap, roadSection, positionInSection, destination, velocity, vmax, amin, amax, hst, hgo, vehicle_length, c, human_reaction, autonomous_reaction)