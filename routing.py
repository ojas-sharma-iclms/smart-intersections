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
import csv

# CLASS DEFINITIONS

class Road:
    def __init__(self, id, orientation, adjacent, length):
        self.id = id
        self.type = "road"
        self.orientation = orientation # eventually the orientation to be displayed on screen when map displayed (string, vertical/horizontal)
        self.length = length
        self.adjacent = adjacent
        self.edges = [(adjacent[0][0], f"{id}.1"), (f"{id}.1", adjacent[0][1]), (adjacent[1][0], f"{id}.0"), (f"{id}.0", adjacent[1][1])]
        self.stack = [[], []]

    def joinSection(self, id, direction):
        self.stack[int(direction)].append(id)

    def leaveSection(self, id, direction):
        if self.stack[int(direction)][0] == id:
            self.stack[int(direction)].pop(0)
        else:
            print(f"something weird has happened when removing {id} from the stack of {self.id}")

class Intersection:
    def __init__(self, id, orientation, adjacent, typeIntersection):
        self.id = id
        self.length = 50 # each section of the intersection is 50m long??
        self.type = "intersection"
        self.orientation = orientation # eventually the orientation to be displayed on screen when map displayed (string, up/down/left/right)
        self.typeIntersection = typeIntersection
        self.adjacent = adjacent
        self.edge_names = [f"{id}.0", f"{id}.1", f"{id}.2", f"{id}.3"]
        self.edges = []
        self.stack = [[], [], [], []]
        self.inStack = [[], [], [], []]
        self.intersectionStack = []

        if self.typeIntersection == "three":
            pass
        elif self.typeIntersection == "four":
            pass

        nodeCounter = 0
        for connection in self.adjacent:
            if nodeCounter != 3:
                self.edges += [(connection[0], self.edge_names[nodeCounter + 1])]
            else:
                self.edges += [(connection[0], self.edge_names[0])]
            self.edges += [(self.edge_names[nodeCounter], connection[1])]
            nodeCounter += 1

        self.newEdges = [(self.edge_names[3], self.edge_names[0]), (self.edge_names[0], self.edge_names[1]), (self.edge_names[1], self.edge_names[2]), (self.edge_names[2], self.edge_names[3])]
        self.edges += self.newEdges

    def joinSection(self, id, direction):
        self.inStack[int(direction)].append(id)

    def leaveSection(self, id, direction):
        if self.inStack[int(direction)][0] == id:
            self.inStack[int(direction)].pop(0)
        else:
            print(f"something weird has happened when removing {id} from the stack of {self.id}")

class EntryExit:
    def __init__(self, id, adjacent):
        self.id = id
        self.length = 0
        self.type = "entry_exit"
        self.adjacent = adjacent
        self.stack = [[], []]

    def joinSection(self, id, direction):
        self.stack[int(direction)].append(id)

    def leaveSection(self, id, direction):
        if self.stack[int(direction)][0] == id:
            self.stack[int(direction)].pop(0)
        else:
            print(f"something weird has happened when removing {id} from the stack of {self.id}")

class RoadNetwork:
    def __init__(self, filename):
        self.mapEmpty = False
        self.roadMap = []
        self.initialRoadMap = [] # constant
        self.points = []
        self.roadNetwork = nx.read_adjlist(f"maps/{filename}.map", comments='#', delimiter=None, create_using=nx.DiGraph, nodetype=None, encoding='utf-8')
        incoming_neighbors = {node: list(self.roadNetwork.predecessors(node)) for node in self.roadNetwork.nodes()}
        outgoing_neighbors = {node: list(self.roadNetwork.successors(node)) for node in self.roadNetwork.nodes()}

        for node in self.roadNetwork:
            id = node.split(".", 1)[0]
            self.points.append(id) if id not in self.points else self.points # compile both lanes / multiple sections of an intersection to one functional object
        for point in self.points:
            if point[0] == "R":
                # road
                self.roadMap.append(Road(point, "horizontal", [[incoming_neighbors[f"{point}.0"],outgoing_neighbors[f"{point}.0"]],[incoming_neighbors[f"{point}.1"],outgoing_neighbors[f"{point}.1"]]], 500))
            elif point[0] == "I":
                # intersection
                tempAdj = []
                if f"{point}.3" in self.roadNetwork:
                    type = "four"
                    num = 4
                else:
                    type = "three"
                    num = 3

                for i in range(num):
                    if i != 3:
                        tempAdj.append([incoming_neighbors[f"{point}.{i}"], outgoing_neighbors[f"{point}.{i+1}"]])
                    else:
                        tempAdj.append([incoming_neighbors[f"{point}.{i}"], outgoing_neighbors[f"{point}.0"]])
                self.roadMap.append(Intersection(point, "up", tempAdj, type))
            elif point[0] == "S":
                # entry/exit
                self.roadMap.append(EntryExit(point, [outgoing_neighbors[f"{point}.1"], incoming_neighbors[f"{point}.0"]]))

        for object in self.roadMap:
            if object.type == "intersection":
                self.roadNetwork.add_edges_from(object.newEdges)
        
        self.routeMap = nx.floyd_warshall(self.roadNetwork)
        self.routeMatrix = {a: dict(b) for a, b in self.routeMap.items()}

    def finaliseRoadMap(self):
        self.initialRoadMap = self.roadMap.copy()

    def getOptimalRoute(self, source, destination):
        path_names = nx.shortest_path(self.roadNetwork, source, destination)
        endNode = path_names[-1]
        path_names.append(endNode) # dodgy
        return path_names
    
    def removeFromMap(self, id):
        self.roadMap.pop(self.lookUp(id))
    
    def getPath(self, id, source, destination):
        path_names = self.getOptimalRoute(source, destination)
        next_object = "CLEAR"
        clear = True
        for node in path_names:
            nodeId = node.split(".", 1)[0]
            dir = node.split(".", 1)[1]
            dir = int(dir)
            stack = self.roadMap[self.lookUp(nodeId)].stack[dir]
            if clear:
                if stack[0] == id:
                    clear = True
                else:
                    clear = False
                    counter = 0
                    for i in stack:
                        if i == id:
                            selfIndex = counter
                        counter += 1
                    next_object = stack[selfIndex - 1] # returns the id of the vehicle's next vehicle
                    next_object_pos = self.roadMap[self.lookUp(next_object)].roadSection
                    if next_object_pos.split(".", 1)[0] != node.split(".", 1)[0]: # make sure next vehicle is not just a pathing, and the vehicle is actually there
                        next_object = node.split(".", 1)[0]
        if next_object == "CLEAR":
            next_object = path_names[-1].split(".", 1)[0]
        return next_object
    
    def plotMapGraph(self):
        pos = nx.kamada_kawai_layout(self.roadNetwork)
        nx.draw(self.roadNetwork, pos, with_labels=True)
        plt.savefig("maps/roadNetwork.png")
        plt.clf()

    def lookUp(self, id):
        counter = 0
        for i in self.roadMap:
            if i.id == id:
                index = counter
            counter += 1
        return index

    def updateStacks(self):
        mapContainsVehicles = False
        for object in self.roadMap:
            if (object.type == "human") or (object.type == "autonomous"): # for each vehicle...
                mapContainsVehicles = True
                route = object.getOptimalRoute()
                for node in route: # for each node on each vehicle's desired path...
                    tempId = node.split(".", 1)[0]
                    tempDir = node.split(".", 1)[1]
                    tempDir = int(tempDir)
                    index = self.lookUp(tempId)
                    if self.roadMap[index].type == "intersection": # add vehicle id to whole intersection stack
                        if (object.id) not in self.roadMap[index].intersectionStack:
                            self.roadMap[index].intersectionStack.append(object.id)
                    if (object.id not in self.roadMap[index].stack[tempDir]): # add vehicle id to stack if necessary
                        self.roadMap[index].stack[tempDir].append(object.id)
                for track in self.roadMap:
                    if ((track.type == "entry_exit") or (track.type == "intersection") or (track.type ==  "road")):
                        tempStack = track.stack
                        counter = 0
                        for i in tempStack:
                            if (object.id in i) and ((f"{track.id}.{counter}") not in route): # remove from stack if necessary
                                i.remove(object.id)
                            counter += 1
                        if track.type == "intersection": # remove from intersection stack if necessary
                            if ((object.id in track.intersectionStack) and (((f"{track.id}.0") or (f"{track.id}.1") or (f"{track.id}.2") or (f"{track.id}.3")) not in route)):
                                track.intersectionStack.remove(object.id)
        if not(mapContainsVehicles):
            self.mapEmpty = True

    def mapStatus(self): # check if all vehicles have left the map
        return self.mapEmpty
    
    def reorderStacks(self):
        for object in self.roadMap:
            if (object.type == "intersection"):
                newStack = [[], [], [], []]
                for section in object.stack:
                    for vehicle in section:
                        # find distance to said intersection
                        pass