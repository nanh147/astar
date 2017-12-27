from math import sqrt
import matplotlib.pyplot as plt
import heapq
from gencircle import *
import random
import numpy as np

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

    def heuristic(self, current, target):
        return sqrt((current.x-target.x)**2 + (current.y-target.y)**2)

    def heuristic2(self, cell):
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    def get_node(self, x, y):
        return self.nodes[x * self.grid_height + y] # they're linear indexed

    def get_neighbours(self, node):
        # CW from right
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
            print node.x, node.y

        path.append((self.start.x, self.start.y))
        path.reverse()
        return path

    def update_node(self, nextNode, node):
        # updates the next node
        nextNode.g = node.g + 10 #self.heuristic(nextNode, self.start)
        nextNode.h = self.heuristic2(nextNode) #self.heuristic2(nextNode) #self.heuristic(nextNode, self.end)
        nextNode.f = nextNode.g + nextNode.h
        nextNode.parent = node

    def process(self):
        heapq.heappush(self.open, (self.start.f, self.start)) # add starting node to top of heap
        while len(self.open):
            f, node = heapq.heappop(self.open) # pop node from queue
            self.closed.add(node) # add to closed

            if node is self.end:
                return self.show_path()

            neighbours = self.get_neighbours(node) # grab the neighbours
            for neighbour in neighbours:
                if not neighbour.obstacle and neighbour not in self.closed:
                    if (neighbour.f, neighbour) in self.open:
                    # if neighbour is in open list, check if current path is better than the one previously found for this neighbour
                        if neighbour.g > node.g + 10: # previously: > node.g + 10
                            self.update_node(neighbour, node)
                    else:
                        self.update_node(neighbour, node)
                        heapq.heappush(self.open, (neighbour.f, neighbour))


if __name__ == '__main__':
# configs
    width = 50
    height = 50
    obstaclePercentage = 2

# generate the obstacles
#     obstacles = np.round(np.random.rand(width*obstaclePercentage,2) * width)
#     obstacles = obstacles.astype(int)
#     obstacles = tuple(map(tuple, obstacles))
    obstacles = gencircle(10, 9,10)
    obstacles = tuple(map(tuple, obstacles))

# plot the obstacles
    xobs, yobs = zip(*obstacles)
    plt.axis([-1, width, -1, height])
    plt.plot(xobs, yobs, 'kx')
    plt.grid()

# run the algorithm
    astar = AStar()
    astar.init_grid([0,0],[width - 1,height - 1], obstacles, width, height)
    result = astar.process()

# plot results
    x, y = zip(*result)
    plt.plot(x,y, 'gd')
    plt.show()





