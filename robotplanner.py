# A* algorithms for search-based planning:
# vanilla A* for algo=0
# A* with node expansionfor algo=1
# A* with node expansion and heuristic update for algo=2

import numpy as np


class environment(object):
    def __init__(self, envmap, robotpos, targetpos, algo=1):
        '''
        class to initialize the environment and store different parameters to ease computation
        algo=0: vanilla A*
        algo=1: A* with node expansion
        algo=2: A* with node expansion and heuristic update
        default algo=1 as it gives fastest results in most maps
        '''
        self.envmap = envmap
        self.numofdirs = 8
        self.dX = [-1, -1, -1, 0, 0, 1, 1, 1]
        self.dY = [-1,  0,  1, -1, 1, -1, 0, 1]
        self.OPEN = []
        self.CLOSED = []
        self.parent = {}
        self.path = []
        self.robotpos = robotpos
        self.targetpos = targetpos
        self.G = np.zeros(self.envmap.shape)
        self.algo = algo
        if self.algo == 2:
            self.lastpos = self.targetpos
            self.H = np.array([[self.getHeuristics(np.array([i, j]), targetpos)
                                for j in range(self.envmap.shape[1])]for i in range(self.envmap.shape[0])])
        else:
            self.H = np.zeros(self.envmap.shape)

    def robotplanner(self, envmap, robotpos, targetpos):
        '''
        main function to execute the algortihm and return the next robot position
        '''
        newrobotpos = np.copy(robotpos)
        if self.algo == 0:
            self.Astar(robotpos, targetpos)
            self.path = self.pathfinder(tuple(robotpos), tuple(targetpos))
        else:
            if not len(self.path):
                self.Astar(robotpos, targetpos)
                temp = self.optimalNode(targetpos)
                self.path = self.pathfinder(tuple(robotpos), tuple(temp))
                if self.algo == 2:
                    if not np.array_equal(self.lastpos, targetpos):
                        self.updateHeuristics(targetpos)
                        self.lastpos = targetpos
        newrobotpos = np.array(self.path.pop())
        return newrobotpos

    def Astar(self, robotpos, targetpos, eps=1, N=500):
        '''
        function to implement A* algorithm
        eps: epsilon parameter to trust heuristics more
        N: number of nodes to expand in the closed list
        default eps=1 and N=500 as it works best for most maps but can be tuned as needed for different cases
        eps=2, N=100 are also good options
        '''
        self.G = np.ones(self.envmap.shape) * np.inf
        self.G[robotpos[0], robotpos[1]] = 0
        self.OPEN = [robotpos.tolist()]
        self.CLOSED = []
        self.parent = {}
        self.path = []
        while targetpos.tolist() not in self.CLOSED:
            hlist = []
            glist = []
            for k in self.OPEN:
                if self.algo != 2:
                    self.H[k[0], k[1]] = np.linalg.norm(
                        np.array(k) - targetpos)
                hlist.append(self.H[k[0], k[1]])
                glist.append(self.G[k[0], k[1]])
            ind = np.argmin(np.array(glist) + eps * np.array(hlist))
            i = self.OPEN.pop(ind)
            if self.algo != 0:
                if len(self.CLOSED) >= N:
                    break
            self.CLOSED.append(i)
            children = self.getChildren(i)
            for j in children:
                if j not in self.CLOSED:
                    if self.algo == 1:
                        cost = 1
                    else:
                        cost = self.cost(i, j)
                    trans = self.G[i[0], i[1]] + cost
                    if self.G[j[0], j[1]] > trans:
                        self.G[j[0], j[1]] = trans
                        self.parent[tuple(j)] = tuple(i)
                        if j not in self.OPEN:
                            self.OPEN.append(j)

    def getChildren(self, robotpos):
        '''
        function to get children of a node
        '''
        children = []
        for dd in range(self.numofdirs):
            newx = robotpos[0] + self.dX[dd]
            newy = robotpos[1] + self.dY[dd]
            if 0 <= newx < self.envmap.shape[0] and 0 <= newy < self.envmap.shape[1]:
                if not self.envmap[newx, newy]:
                    children.append([newx, newy])
        return children

    def optimalNode(self, targetpos):
        '''
        function to get the optimal node for the robot position
        '''
        hnew = []
        gnew = []
        for k in self.OPEN:
            if np.array_equal(k, targetpos):
                return k
            if self.algo == 1:
                self.H[k[0], k[1]] = np.linalg.norm(np.array(k) - targetpos)
            hnew.append(self.H[k[0], k[1]])
            gnew.append(self.G[k[0], k[1]])
        comp = np.array(gnew) + np.array(hnew)
        ind = np.argmin(comp)
        if self.algo == 2:
            val = comp[ind]
            for k in self.CLOSED:
                self.H[k[0], k[1]] = val - self.G[k[0], k[1]]
        return self.OPEN[ind]

    def getHeuristics(self, node, targetpos):
        '''
        function to get heuristic values for a node in the form of Euclidean distance
        '''
        return np.linalg.norm(np.array(node) - targetpos)

    def updateHeuristics(self, targetpos):
        for i in range(self.envmap.shape[0]):
            for j in range(self.envmap.shape[1]):
                self.H[i, j] = max(self.getHeuristics(np.array(
                    [i, j]), targetpos), self.H[i, j] - self.H[targetpos[0], targetpos[1]])
        self.H = np.array(self.H)

    def cost(self, oldpos, newpos):
        '''
        function to calculate cost of transition between nodes
        '''
        if newpos[0] != oldpos[0] and newpos[1] != oldpos[1]:
            return np.sqrt(2)
        else:
            return 1

    def pathfinder(self, robotpos, targetpos):
        '''
        function to generate optimal path from start to target from parent dictionary
        '''
        loc = [targetpos]
        temp = targetpos
        while not np.array_equal(temp, robotpos):
            temp = self.parent[temp]
            loc.append(temp)
        loc.pop()
        return loc
