# myGhostAgents.py

from game import Agent
from game import Actions
from game import Directions
import random
from util import Queue, matrixAsList
from util import manhattanDistance
import util
import numpy as np


def shortest_path(walls, start,end):  # can access walls from agents state.getWalls() gives numpy 2D array, true = walls, false = space
    start = [start[0], start[1]]
    end = [end[0], end[1]]
    if start == end:
        return [start]
    neighbours = Queue()  # queue storing the next positions to explore
    neighbours.push(start)
    counts = np.zeros((walls.width, walls.height),dtype=int)  # 2D array to store the distance from the start to all visted points
    predecessors = np.zeros((counts.shape[0], counts.shape[1], 2),dtype=int)  # 2D array storing the predecessors (past points allowing path to be retraced)
    counts[start[0], start[1]] = 1
    # loop until the end position is found
    while not neighbours.isEmpty():
        n = neighbours.pop()
        #print n
        if n == end:
            #print "path found!", n, counts[n[0]][n[1]]
            break  # path found
        # add all the valid neighbours to the list and remember from where they came from
        for neighbour in [[n[0] - 1, n[1]], [n[0] + 1, n[1]], [n[0], n[1] - 1], [n[0], n[1] + 1]]:
            if not walls[int(neighbour[0])][int(neighbour[1])] and counts[int(neighbour[0])][int(neighbour[1])] == 0:
                neighbours.push(neighbour)
                predecessors[neighbour[0], neighbour[1]] = n
                counts[neighbour[0], neighbour[1]] = counts[n[0], n[1]] + 1
    #print 'here 0'
    if counts[end[0], end[1]] == 0:
        return []  # path not found
    #print 'here 1'
    path = []
    n = end
    #print end, counts[end[0], end[1]]
    # reconstruct the path
    #print counts
    while n != start:
        # print n
        if n == start:
            break
        path.append(n)
        n = predecessors[n[0], n[1]].tolist()
    path.append(start)
    #print 'here 2'
    #print counts

    return path


class ManhattanGhost(Agent):
    def __init__( self, index ):
        self.index = index

    def getAction(self, state):
        dist = self.getDistribution(state)
        if len(dist) == 0:
            return Directions.STOP
        else:
            return util.chooseFromDistribution(dist)

    def __init__(self, index, prob_abandonpost=0.9, prob_guard=0.9):
        self.index = index
        self.prob_guard = prob_guard #probability of taking best action when it's guarding the capsule
        self.prob_abandonpost = prob_abandonpost #probability of taking best action when it's not guarding the capsule



    def getDistribution(self, state):
        # Read variables from state
        ghostState = state.getGhostState(self.index)
        legalActions = state.getLegalActions(self.index)

        pos=state.getGhostPosition(self.index)

        isScared = ghostState.scaredTimer > 0

        speed = 1
        if isScared: speed = 0.5

        actionVectors = [Actions.directionToVector(a, speed) for a in legalActions]
        newPositions = [(pos[0] + a[0], pos[1] + a[1]) for a in actionVectors]
        pacmanPosition = state.getPacmanPosition()
        capsulePosition = state.getCapsules() #get list of existing capsule locations
        #locate capsule

        # Select best actions given the state
        distancesToPacman = [manhattanDistance(pos, pacmanPosition) for pos in newPositions] #finds the distance from ghost to pacman after tking the next step
        if self.index>len(capsulePosition): #checks how many ghosts are left
            distancesToGoal=distancesToPacman #makes pacman the ghosts goal
            pac_to_cap=1 #treat it as if pacman is close to capsule
        else:
            distancesToGoal = [manhattanDistance(pos, capsulePosition[self.index-1]) for pos in newPositions] #makes capsule the ghosts goal
            pac_to_cap= manhattanDistance(capsulePosition[self.index-1], pacmanPosition)

        ghost_to_goal= min(distancesToGoal) #shortest distance to the goal (pacman or capsule) after step is taken

        d = manhattanDistance(pos, pacmanPosition) #current distanc to pacman

        if isScared:
            if d > 10: #if pacman is farther than safety zone (10)
                safe = True
            else:
                safe = False
        else:
            if ghost_to_goal < pac_to_cap or d>4: safe = True #ghost is safe if it's closer to capsule than pacman or when pacman is far away
            else: safe=False #ghost is not safe if pacman is closer to capsule and pacman is close

        if safe:
            if isScared or d>2: #do this when ghost is safe and scared or safe and pacman is far
                bestScore = min(distancesToGoal) #keep aiming for goal
                bestProb = self.prob_guard
                bestActions = [action for action, distance in zip(legalActions, distancesToGoal) if
                           distance == bestScore]
            else: #do this if ghost is safe and not scared and pacman is close
                bestScore = min(distancesToPacman) #chase pacman
                bestProb = self.prob_abandonpost
                bestActions = [action for action, distance in zip(legalActions, distancesToPacman) if
                               distance == bestScore]
        else: #do this when not safe
            bestScore = max(distancesToPacman)
            bestProb = self.prob_abandonpost
            bestActions = [action for action, distance in zip(legalActions, distancesToPacman) if distance == bestScore]



        # Construct distribution
        dist = util.Counter()
        for a in bestActions: dist[a] = bestProb / len(bestActions)
        #for a in legalActions: dist[a] += (1 - bestProb) / len(legalActions)
        dist.normalize()
        return dist

class GuardGhost(Agent):
    def __init__( self, index ):
        self.index = index

    def getAction(self, state):
        dist = self.getDistribution(state)
        if len(dist) == 0:
            return Directions.STOP
        else:
            return util.chooseFromDistribution(dist)

    def __init__(self, index, prob_abandonpost=0.9, prob_guard=0.9):
        self.index = index
        self.prob_guard = prob_guard #probability of taking best action when it's guarding the capsule
        self.prob_abandonpost = prob_abandonpost #probability of taking best action when it's not guarding the capsule



    def getDistribution(self, state):
        # Read variables from state
        ghostState = state.getGhostState(self.index)
        legalActions = state.getLegalActions(self.index)
        walls = state.getWalls()
        pos=state.getGhostPosition(self.index)

        isScared = ghostState.scaredTimer > 0

        speed = 1
        if isScared: speed = 0.5

        actionVectors = [Actions.directionToVector(a, speed) for a in legalActions]
        newPositions = [(pos[0] + a[0], pos[1] + a[1]) for a in actionVectors]
        pacmanPosition = state.getPacmanPosition()
        capsulePosition = state.getCapsules() #get list of existing capsule locations


        # Select best actions given the state
        distancesToPacman = [len(shortest_path(walls, pos, pacmanPosition)) for pos in newPositions] #finds the distance from ghost to pacman after tking the next step

        if self.index>len(capsulePosition): #checks how many capsules are left
            distancesToGoal=distancesToPacman #makes pacman the ghosts goal
            pac_to_cap=1 #treat it as if pacman is close to capsule
        else:
            distancesToGoal = [len(shortest_path(walls, pos, capsulePosition[self.index-1])) for pos in newPositions] #makes capsule the ghosts goal
            pac_to_cap=len(shortest_path(walls, capsulePosition[self.index-1], pacmanPosition))

        print distancesToGoal
        ghost_to_goal= min(distancesToGoal) #shortest distance to the goal (pacman or capsule) after step is taken
        d = len(shortest_path(walls, pos, pacmanPosition)) #current distance to pacman

        if isScared:
            if d > 10: #if pacman is farther than safety zone (10)
                safe = True
            else:
                safe = False
        else:
            if ghost_to_goal < pac_to_cap or d>4: safe = True #ghost is safe if it's closer to capsule than pacman or when pacman is far away
            else: safe=False #ghost is not safe if pacman is closer to capsule and pacman is close

        if safe:
            if isScared or d>2: #do this when ghost is safe and scared or safe and pacman is far
                bestScore = min(distancesToGoal) #keep aiming for goal
                bestProb = self.prob_guard
                bestActions = [action for action, distance in zip(legalActions, distancesToGoal) if
                           distance == bestScore]
            else: #do this if ghost is safe and not scared and pacman is close
                bestScore = min(distancesToPacman) #chase pacman
                bestProb = self.prob_abandonpost
                bestActions = [action for action, distance in zip(legalActions, distancesToPacman) if
                               distance == bestScore]
        else: #do this when not safe
            bestScore = max(distancesToPacman)
            bestProb = self.prob_abandonpost
            bestActions = [action for action, distance in zip(legalActions, distancesToPacman) if distance == bestScore]



        # Construct distribution
        dist = util.Counter()
        for a in bestActions: dist[a] = bestProb / len(bestActions)
        #for a in legalActions: dist[a] += (1 - bestProb) / len(legalActions)
        dist.normalize()
        return dist