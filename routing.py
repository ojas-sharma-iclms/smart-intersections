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
        pass

    def leaveSection(self, id, direction):
        if self.stack[int(direction)][0] == id:
            self.stack[int(direction)].pop(0)
        else:
            print(f"something weird has happened when removing {id} from the stack of {self.id}")
        pass

class Intersection:
    def __init__(self, id, orientation, adjacent, typeIntersection):
        self.id = id
        self.type = "intersection"
        self.orientation = orientation # eventually the orientation to be displayed on screen when map displayed (string, up/down/left/right)
        self.typeIntersection = typeIntersection
        self.adjacent = adjacent
        self.edge_names = [f"{id}.0", f"{id}.1", f"{id}.2", f"{id}.3"]
        self.edges = []
        self.stack = [[], [], [], []]

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

class EntryExit:
    def __init__(self, id, adjacent):
        self.id = id
        self.type = "entry_exit"
        self.adjacent = adjacent
        self.stack = [[], []]

class RoadNetwork:
    def __init__(self, filename):
        self.roadMap = []
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
                self.roadMap.append(Road(point, "horizontal", [[incoming_neighbors[f"{point}.0"],outgoing_neighbors[f"{point}.0"]],[incoming_neighbors[f"{point}.1"],outgoing_neighbors[f"{point}.1"]]], 50))
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

    def getOptimalRoute(self, source, destination):
        path_names = nx.shortest_path(self.roadNetwork, source, destination)
        return path_names
    
    def plotMapGraph(self):
        pos = nx.kamada_kawai_layout(self.roadNetwork)
        nx.draw(self.roadNetwork, pos, with_labels=True)
        plt.savefig("roadNetwork.png")
        plt.clf()

    def lookUp(self, id):
        counter = 0
        for i in self.roadMap:
            if i.id == id:
                index = counter
            counter += 1
        return index

    def updateStacks(self):
        for object in self.roadMap:
            if (object.type == "human") or (object.type == "autonomous"): # for each vehicle...
                route = object.getOptimalRoute()
                for node in route: # for each node on each vehicle's desired path...
                    tempId = id = node.split(".", 1)[0]
                    tempDir = id = node.split(".", 1)[1]
                    tempDir = int(tempDir)
                    index = self.lookUp(tempId)
                    if (object.id not in self.roadMap[index].stack[tempDir]): # add vehicle id to stack if necessary
                        self.roadMap[index].stack[tempDir].append(object.id)
                for track in self.roadMap:
                    if ((track.type == "entry_exit") or (track.type == "intersection") or (track.type ==  "road")):
                        tempStack = track.stack
                        counter = 0
                        for i in tempStack:
                            if (object.id in i) and ((f"{track.id}.{counter}") not in route):
                                i.remove(object.id)
                                counter += 1