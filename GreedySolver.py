import copy
import heapq

import numpy as np

import TSPClasses
from TSPClasses import Scenario
from TSPClasses import TSPSolution


class GreedySolver:
    def __init__(self):
        pass

    def solve(self, scenario: Scenario):
        adjList = []

        for departingCity in scenario.getCities():  # build adjacency list
            neighborsHeap = []
            arrivalCityIdx = 0
            for arrivalCity in scenario.getCities():
                dist = departingCity.costTo(arrivalCity)
                neighborsHeap.append((dist, id(arrivalCity), arrivalCity, arrivalCityIdx))
                arrivalCityIdx += 1

            heapq.heapify(neighborsHeap)
            adjList.append(neighborsHeap)

        solFound = False
        path = []
        visited = set()
        originIdx = 0
        currCost = 0

        while not solFound:
            currIdx = originIdx

            while len(path) < len(scenario.getCities()):
                currCity = scenario.getCities()[currIdx]
                path.append(currCity)
                visited.add(currCity)
                if len(path) == len(scenario.getCities()):
                    break

                closestNeighbor = heapq.heappop(adjList[currIdx])
                while closestNeighbor[2] in visited:  # pop until get an unvisited city
                    closestNeighbor = heapq.heappop(adjList[currIdx])

                dist = closestNeighbor[0]

                if dist == np.inf:  # dead end. starting over
                    originIdx += 1
                    if originIdx == len(adjList):  # gone thru all vertices without a sol
                        return None
                    path.clear()
                    visited.clear()
                    currCost = 0
                    break

                currCost += dist

                currIdx = closestNeighbor[3]

            if len(path) < len(scenario.getCities()):
                continue

            solution: TSPSolution = TSPSolution(path)
            cost = solution.cost
            if cost == np.inf:
                continue

            return solution














