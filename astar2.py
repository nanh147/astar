from math import sqrt
import matplotlib.pyplot as plt
import heapq
from gencircle import *
import numpy as np
import timeit
# adapted from: https://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/

class Node(object):
    def __init__(self, x, y, obstacle):
        self.obstacle = obstacle;
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0 # node-start
        self.h = 0 # node-target
        self.f = 0 # g + h

class AStar(object):
    def __init__(self):
        self.open = []
        heapq.heapify(self.open)
        self.closed = set()
        self.nodes = []
        self.grid_height = None
        self.grid_width = None
        self.heuristic_type = 1 # 1 = pythagoras, 2 = manhattan distance

    def init_grid(self, start, end, obstacles, width, height):
        self.grid_width = width
        self.grid_height = height

        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if (x, y) in obstacles:
                    obstacle = True
                else:
                    obstacle = False
                self.nodes.append(Node(x, y, obstacle))

        self.start = self.get_node(*start)
        self.end = self.get_node(*end)

    def init_grid2(self, start, end, obstacles, width, height):
        positions = self.genPoints(width, height, obstacles)
        self.grid_width = width
        self.grid_height = height

        for row in positions:
            self.nodes.append(Node(row[0], row[1], 0))

        for row in obstacles:
            self.nodes.append(Node(row[0], row[1], 1))

        self.nodes.sort(key=lambda x: x.y)
        self.nodes.sort(key=lambda x: x.x)

        # self.nodes= tuple(self.nodes(tuple, obstacles)) # convert to tuple

        self.start = self.get_node(*start)
        self.end = self.get_node(*end)


    def genPoints(self,width, height, obstacles):
        x, y = np.mgrid[0:width, 0:height]
        positions = np.column_stack([x.ravel(),y.ravel()])  # creates x and y columns: https://stackoverflow.com/questions/12864445/numpy-meshgrid-points/12891609

        for row in obstacles:
            admissable = np.not_equal(positions, row)
            admissable = np.logical_or(admissable[:, 0], admissable[:,1])  # required because not_equal does element wise, not row wise. Blocks off rows that entirely match obstacle
            positions = positions[admissable]

        return positions

    def heuristic(self, current, target): # pythag distance
        return sqrt((current.x-target.x)**2 + (current.y-target.y)**2)

    def heuristic2(self, cell): # manhattan distance
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    def get_node(self, x, y):
        return self.nodes[x * self.grid_height + y] # they're linear indexed

    def get_neighbours(self, node):
        neighbours = []
        if node.x < self.grid_width - 1:
            neighbours.append(self.get_node(node.x + 1, node.y))
        if node.y > 0:
            neighbours.append(self.get_node(node.x, node.y - 1))
        if node.x > 0:
            neighbours.append(self.get_node(node.x - 1, node.y))
        if node. y < self.grid_height - 1:
            neighbours.append(self.get_node(node.x, node.y + 1))
        if node.y > 0 and node.y < self.grid_height -1 and node.x < self.grid_width-1: # up, right
            neighbours.append(self.get_node(node.x + 1, node.y + 1))
        if node.y > 0 and node.y < self.grid_height -1 and node.x > 0: # up, left
            neighbours.append(self.get_node(node.x - 1, node.y + 1))
        if node.y > 1 and node.y < self.grid_height and node.x < self.grid_width -1: # down, right # check if I can extend grid height by 1
            neighbours.append(self.get_node(node.x + 1, node.y - 1))
        if node.y > 1 and node.y < self.grid_height and node.x > 0: # down, left
            neighbours.append(self.get_node(node.x - 1, node.y - 1))

        return neighbours

    def show_path(self):
        node = self.end
        path = []
        path.append((node.x, node.y))
        while node.parent is not self.start:
            node = node.parent
            path.append((node.x, node.y))
            # print node.x, node.y

        path.append((self.start.x, self.start.y))
        path.reverse()
        return path

    def update_node(self, nextNode, node):
        # updates the next node
        if self.heuristic_type is 1: # pythagoras
            nextNode.g = self.heuristic(nextNode, self.start)
            nextNode.h = self.heuristic(nextNode, self.end)

        elif self.heuristic_type is 2:
            nextNode.g = node.g + 10 # manhattan distance
            nextNode.h = self.heuristic2(nextNode)  # self.heuristic2(nextNode) #self.heuristic(nextNode, self.end)

        nextNode.f = nextNode.g + nextNode.h
        nextNode.parent = node

    def process(self):
        heapq.heappush(self.open, (self.start.f, self.start)) # add starting node to top of heap
        while len(self.open):
            f, node = heapq.heappop(self.open) # pop node from queue
            self.closed.add(node) # add to closed

            if node is self.end: # finished, find the path
                return self.show_path()

            neighbours = self.get_neighbours(node) # grab current node's neighbours
            for neighbour in neighbours:
                if not neighbour.obstacle and neighbour not in self.closed:
                    if (neighbour.f, neighbour) in self.open:

                        # of the neighbours, select one with the optimal heuristic depending on which heuristic is being used
                        if self.heuristic_type is 1:
                            if neighbour.h < node.h:
                                self.update_node(neighbour, node)
                        elif self.heuristic_type is 2:                     # if neighbour is in open list, check if current path is better than the one previously found for this neighbour
                            if neighbour.g > node.g + 10:
                                self.update_node(neighbour, node)
                    else:
                        self.update_node(neighbour, node)
                        heapq.heappush(self.open, (neighbour.f, neighbour))
        print 'died'
def genObstacles(num_obstacles, width, height, circle = 1):
    # randomly distributed point obstacles
    if not circle:
        obstacles = np.round(np.random.rand(num_obstacles,2) * [width, height])
        obstacles = obstacles.astype(int)

        # remove out of range obstacles
        obstacles = obstacles[obstacles[:,0] >= 0] # x GTE 0
        obstacles = obstacles[obstacles[:,0] < width] # x LT width
        obstacles = obstacles[obstacles[:,1] >= 0] # y GTE 0
        obstacles = obstacles[obstacles[:,1] < height] # y LT height
        return tuple(map(tuple, obstacles))

    # randomly placed circular objects
    else:
        obstacle_locs = np.round(np.random.rand(num_obstacles, 2) * width)
        obstacles = np.empty([1, 2])
        for row in obstacle_locs:
            obstacles = np.vstack((obstacles, gencircle(10, row[0], row[1])))

        # strip out obstacles that are out of range
        obstacles = obstacles[obstacles[:,0] >= 0] # x GTE 0
        obstacles = obstacles[obstacles[:,0] < width] # x LT width
        obstacles = obstacles[obstacles[:,1] >= 0] # y GTE 0
        obstacles = obstacles[obstacles[:,1] < height] # y LT height

        obstacles = np.unique(obstacles, axis = 0)

        obstacles = obstacles.astype(int)
        return tuple(map(tuple, obstacles)) # convert to tuple


if __name__ == '__main__':
# configs
    width = 100
    height = 200
    num_obstacles = 200
    circular_obstacles = False # False: randomly placed point obstacles

    obstacles = genObstacles(num_obstacles,width,height,circular_obstacles)

# plot the obstacles
    xobs, yobs = zip(*obstacles)
    plt.axis([-1, width, -1, height])
    plt.plot(xobs, yobs, 'kx')
    plt.grid()

# run the algorithm
    print 'Algorithm'
    astar = AStar()

    start_time = timeit.default_timer()

    astar.init_grid2([0,0], [width - 1,height - 1], obstacles, width, height)
    elapsed = timeit.default_timer() - start_time
    print elapsed


    print 'Process start'
    result = astar.process()

# plot results
    x, y = zip(*result)
    plt.plot(x,y, '-gd')
    plt.show()





