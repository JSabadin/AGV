#!/usr/bin/python3
# -*- coding: utf-8 -*-
from graph_gen import tagMap, tagDets
import heapq
import numpy as np

class PathPlanning(object):
    def __init__(self):
        pass

    def heuristic(self, a, b):
        '''Calculate a refined heuristic.'''
        # Use the minimum of the Euclidean distances from each neighbor of 'a' to 'b'
        neighbors = [tagMap[a][i] for i in range(0, len(tagMap[a]), 2) if tagMap[a][i] != 0]
        if not neighbors:
            return float('inf')
        return min(((tagDets[n][0] - tagDets[b][0]) ** 2 + (tagDets[n][1] - tagDets[b][1]) ** 2) ** 0.5 for n in neighbors)

    def findPath(self, startId, goalId):
        '''Find the shortest path using A* algorithm.'''

        openSet = [(0, startId)]
        heapq.heapify(openSet)

        gScore = {node: float('inf') for node in tagMap}
        gScore[startId] = 0

        fScore = {node: float('inf') for node in tagMap}
        fScore[startId] = self.heuristic(startId, goalId)

        fScoreDict = {}  # Dictionary to store lists of fScores for each node
        cameFrom = {}

        while openSet:
            fScoreCurrent, current = heapq.heappop(openSet)

            # DEBUG statement
            # if current in fScoreDict:
            #     print(f"Current node: {current}, Current fScore: {fScoreCurrent}, Recorded fScores: {fScoreDict[current]}")

            # Skip the current node if its fScore is higher than any previously recorded fScore for this node
            if current in fScoreDict and any(fScoreCurrent > past_fScore for past_fScore in fScoreDict[current] if fScoreCurrent != past_fScore):
                print(f"Node {current} has a duplicate node with lower fScore -> We skip the search for node {current} and pop it out of the queue")
                continue

             # If the goal is reached, reconstruct and return the path.
            if current == goalId:
                path = []
                while current in cameFrom:
                    path.append(current)
                    current = cameFrom[current]
                path.append(startId)
                return path[::-1]

            neighbors = [(tagMap[current][i], tagMap[current][i+1]) for i in range(0, len(tagMap[current]), 2) if tagMap[current][i] != 0]
            for neighbor, distance in neighbors:
                tentative_gScore = gScore[current] + distance
                 # If this path to neighbor is better, record it.
                if tentative_gScore < gScore[neighbor]:
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentative_gScore
                    fScore[neighbor] = tentative_gScore + self.heuristic(neighbor, goalId)
                    
                    # Append the new fScore for this neighbor or create a new list if this neighbor is not in fScoreDict
                    if neighbor in fScoreDict:
                        fScoreDict[neighbor].append(fScore[neighbor])
                    else:
                        fScoreDict[neighbor] = [fScore[neighbor]]
                    # Add the neighbor to the open set
                    heapq.heappush(openSet, (fScore[neighbor], neighbor))

        return []  # Return empty path if no path is found
    



    def generateActions(self, path):
        '''Generate a list of actions for given path
        
        Inputs:
          path - A list of ordered tags that lead from the start tag to the goal tag
                (including start and goal tag) or an empty list if path is not found.

        Outputs:
          actions - A list of actions the AGV need to execute in order to reach the goal tag
                    from the start tag or an empty list if no action is required/possible.
        '''
        actions = []
        for idx, node in enumerate(path):
            if idx >= len(path) - 1:
                continue
            next_idx = tagMap[node].index(path[idx + 1])
            if next_idx in [0, 2, 4]:
                direction = "left" if next_idx == 0 else "right" if next_idx == 2 else "straight"
                distance = tagMap[node][next_idx + 1]
                actions.append((direction, path[idx + 1], distance))

        return actions



if __name__ == '__main__':
  pp = PathPlanning()
  path = pp.findPath(7, 107)
  print(path)
  actions = pp.generateActions(path)
  print(actions)



















###################################### OLD CODE #####################################################
# from graph_gen import tagMap, tagDets
# import heapq

# class PathPlanning(object):
#     def __init__(self):
#         pass

#     def heuristic(self, a, b):
#         '''Calculate a refined heuristic.'''
#         # Use the minimum of the Euclidean distances from each neighbor of 'a' to 'b'
#         neighbors = [tagMap[a][i] for i in range(0, len(tagMap[a]), 2) if tagMap[a][i] != 0]
#         if not neighbors:
#             return float('inf')
#         return min(((tagDets[n][0] - tagDets[b][0]) ** 2 + (tagDets[n][1] - tagDets[b][1]) ** 2) ** 0.5 for n in neighbors)

#     def findPath(self, startId, goalId):
#         '''Find the shortest path using A* algorithm.'''

#         openSet = [(0, startId)]
#         heapq.heapify(openSet)

#         gScore = {node: float('inf') for node in tagMap}
#         gScore[startId] = 0

#         fScore = {node: float('inf') for node in tagMap}
#         fScore[startId] = self.heuristic(startId, goalId)

#         cameFrom = {}

#         while openSet:
#             current = heapq.heappop(openSet)[1]

#             if current == goalId:
#                 path = []
#                 while current in cameFrom:
#                     path.append(current)
#                     current = cameFrom[current]
#                 path.append(startId)
#                 return path[::-1]

#             neighbors = [(tagMap[current][i], tagMap[current][i+1]) for i in range(0, len(tagMap[current]), 2) if tagMap[current][i] != 0]
#             for neighbor, distance in neighbors:
#                 tentative_gScore = gScore[current] + distance
#                 if tentative_gScore < gScore[neighbor]:
#                     cameFrom[neighbor] = current
#                     gScore[neighbor] = tentative_gScore
#                     fScore[neighbor] = tentative_gScore + self.heuristic(neighbor, goalId)
#                     heapq.heappush(openSet, (fScore[neighbor], neighbor))

#         return []  # return empty path if no path is found