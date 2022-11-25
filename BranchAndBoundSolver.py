import time

import TSPClasses
from TSPClasses import Scenario
from State import State
import copy
import heapq
from queue import PriorityQueue
import numpy as np


class BrandAndBoundSolver:
    def __init__(self):
        self.numPrunedStates = 0
        self.maxQSize = 1
        self.totalStates = 1
        self.numSolutions = 0
        self.currBestState: State = None

        self.timeElapsed = 0
        self.solutionFound = False
        self.bestCostSoFar = np.inf
        pass

    def solve(self, scenario: Scenario, time_allowance, startTime):

        numPruned = 0
        initialState: State = State(scenario)
        initialState.reduce()

        stateQ = [(initialState.lowerBound, -len(initialState.blockedCols), id(initialState), initialState)]
        # prioritize the state with more cities visited when two states have equal bounds
        # id helps handle when 2 states have equal bounds and numVisited

        while len(stateQ) != 0:
            self.maxQSize = max(self.maxQSize, len(stateQ))
            currState: State = heapq.heappop(stateQ)[3]

            unvisitedCities: list[int] = currState.getUnvisitedCities()

            if len(unvisitedCities) == 0:  # go back to the first city
                row = currState.path[len(currState.path) - 1]
                col = currState.path[0]
                currState.lowerBound += currState.matrix[row][col]
                currState.matrix[row][col] = np.inf

                if not self.__shouldPrune(currState):
                    self.currBestState = currState
                    self.bestCostSoFar = currState.lowerBound
                    self.solutionFound = True
                    self.numSolutions += 1
                else:
                    numPruned += 1

            for city in unvisitedCities:
                self.timeElapsed = time.time() - startTime
                if self.timeElapsed >= time_allowance:
                    return

                childState: State = currState.clone()
                self.totalStates += 1

                row = childState.path[len(childState.path) - 1]
                col = city
                childState.path.append(city)
                childState.lowerBound += childState.matrix[row][col]

                if self.__shouldPrune(childState):
                    self.numPrunedStates += 1
                    continue

                childState.block(row, col)
                childState.setVisited(col)
                childState.reduce()

                if self.__shouldPrune(childState):
                    self.numPrunedStates += 1
                    continue

                heapq.heappush(stateQ,
                               (childState.lowerBound, -len(childState.blockedCols), id(childState), childState))

            self.timeElapsed = time.time() - startTime

    def __shouldPrune(self, state: State):
        if state.lowerBound == np.inf:
            return True
        if state.lowerBound > self.bestCostSoFar:
            return True

        return False
