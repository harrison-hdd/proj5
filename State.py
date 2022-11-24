import copy
import heapq
import queue
from queue import PriorityQueue

import numpy as np
from numpy import ndarray

import TSPClasses
from TSPClasses import *


class State:
    def __init__(self, scenario: Scenario):
        self.lowerBound = 0
        self.matrix = []
        self.path = []
        self.path = [0]  # start from city 0
        self.visited = []
        self.blockedRows = set()
        self.blockedCols = set()
        self.__fillMatrix(scenario.getCities())
        self.__fillVisited()

    def __fillMatrix(self, cities: list[City]):

        for i in range(0, len(cities)):
            row: list = []
            for j in range(0, len(cities)):
                cost = cities[i].costTo(cities[j])
                row.append(cost)

            self.matrix.append(row)

        self.matrix = ndarray(shape=(len(cities), (len(cities))), buffer=np.array(self.matrix))

    def __fillVisited(self):
        self.visited.append(True)
        for i in range(1, len(self.matrix)):
            self.visited.append(False)

    def getUnvisitedCities(self) -> list[int]:
        unvisitedCities = []
        for i in range(0, len(self.visited)):
            if not self.visited[i]:
                unvisitedCities.append(i)

        return unvisitedCities

    def setVisited(self, city: int):
        self.visited[city] = True

    def reduce(self):
        self.__reduceRows(self.blockedRows)  # reduce rows

        self.matrix = np.transpose(self.matrix)  # constant time

        self.__reduceRows(self.blockedCols)  # reduce columns (now rows in transposed matrix)

        self.matrix = np.transpose(self.matrix)  # constant time

    def __reduceRows(self, blockRows: set):
        for i in range(0, len(self.matrix)):  # reduce rows
            if i in blockRows:
                continue
            minVal = min(self.matrix[i])

            self.lowerBound += minVal

            if minVal == np.inf:
                continue

            for j in range(0, len(self.matrix[i])):
                self.matrix[i][j] -= minVal

    def block(self, row, col):
        self.__blockRows(row)
        self.blockedRows.add(row)

        self.matrix = np.transpose(self.matrix)
        self.__blockRows(col)
        self.matrix = np.transpose(self.matrix)
        self.blockedCols.add(col)

        self.matrix[col][row] = np.inf


    def __blockRows(self, row):
        for i in range(0, len(self.matrix)):
            self.matrix[row][i] = np.inf

    def clone(self):
        return copy.deepcopy(self)

    def __le__(self, other):
        return self.lowerBound <= other.lowerBound

# if __name__ == '__main__':
#     heap = np.array([9, 3, 5, 6,2 ,8 ,4 , 0, 10, np.inf]).tolist()
#
#     heapq.heapify(heap)
#
#     heap

# heap = []
# heap[0] = 1


#     print(np.inf < np.inf)
# q = []
#
# s1 = State()
# s1.lowerBound = 2
#
# s2 = State()
# s2.lowerBound = 1
#
# s3 = State()
# s3.lowerBound = 0
#
# q.append((s1.lowerBound, s1))
# q.append((s2.lowerBound, s2))
# q.append((s3.lowerBound, s3))
#
# # heapq.heappush(q, (s1.lowerBound, s1))
# # heapq.heappush(q, (s2.lowerBound, s2))
# # heapq.heappush(q, (s3.lowerBound, s3))
#
# heapq.heapify(q)
# a = heapq.heappop(q)
#
# print(a)
#
# pass
