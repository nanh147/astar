from math import sqrt
import matplotlib.pyplot as plt
import heapq
import numpy as np
from geo_coords import *
from obstacle import *
from numba import jit
# example of image on map plot: http://scitools.org.uk/cartopy/docs/latest/matplotlib/advanced_plotting.html

# adapted from: https://www.laurentluce.com/posts/solving-mazes-using-python-simple-recursivity-and-a-search/
#check
class Node(object):
    def __init__(self, x, y, lat, lon, obstacle):
        self.obstacle = obstacle
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0 # node-start
        self.h = 0 # node-target
        self.f = 0 # g + h
        self.lat = lat
        self.lon = lon

class AStar(object):
    def __init__(self):
        self.open = []
        heapq.heapify(self.open)
        self.closed = set()
        self.nodes = []
        self.grid_height = None
        self.grid_width = None
        # todo: use equivalent of function pointers to make the line below cleaner
        self.heuristic_type = 1 # 1 = pythagoras, 2 = manhattan distance
        self.geo_path = []
        self.grid_path = []
        self.obstacles = []

    def init_grid3(self, start, end, obstacles, geo_coords, width, height):
        self.grid_width = width
        self.grid_height = height
        self.obstacles = obstacles

# Use KD tree to find xy coordinates that match obstacle locations (distance = 0), set obstacle flag
        xyTree = scipy.spatial.cKDTree(geo_coords[:, 0:2]) # tree in x,y columns
        dists, obstacle_indices = xyTree.query(obstacles)  # all of the obstacle locs actually on the grid will return zero for distance
        obstacle_indices = obstacle_indices[dists == 0.0]  # retrieve only indices for obstacle points on the grid

        geo_coords[obstacle_indices,4] = 1 # flag obstacles

        # create the list of nodes
        self.nodes = [Node(int(row[0]), int(row[1]), row[2], row[3], row[4]) for row in geo_coords]

        self.start = self.get_node(*start)
        self.end = self.get_node(*end)

    def gen_points(self, width, height, obstacles):
        x, y = np.mgrid[0:width, 0:height]
        # creates x and y columns: https://stackoverflow.com/questions/12864445/numpy-meshgrid-points/12891609
        positions = np.column_stack([x.ravel(),y.ravel()])

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

    @jit
    def show_path(self):
        node = self.end
        path = []
        self.grid_path.append((node.x, node.y))
        self.geo_path.append((node.lat, node.lon))

        while node.parent is not self.start:
            node = node.parent
            self.grid_path.append((node.x, node.y))
            self.geo_path.append((node.lat, node.lon))
            # print node.x, node.y

        self.grid_path.append((self.start.x, self.start.y))
        self.geo_path.append((self.start.lat, self.start.lon))

        self.grid_path.reverse()
        self.geo_path.reverse()
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

    @jit
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

    def plot_all(self):
        xobs, yobs = zip(*self.obstacles)
        plt.axis([-1, self.grid_width, -1, self.grid_height])
        plt.plot(xobs, yobs, 'ko')
        plt.grid()

        # Plot path
        x, y = zip(*self.grid_path)
        plt.plot(x, y, '-gd')
        plt.show()

if __name__ == '__main__':
# configs
    num_obstacles = 5 # With very rectangular grids, many obstacles will be out of range so you'll get fewer than this

    geo = GeoCoords([49.128397,-122.796805], [49.129779,-122.790330],2) # flight bounds (SW, NE), spatial resolution

    # obstacles = genTestObstaclesGeo(5, geo, 10)
    obstacles = genObstaclesGeo(geo, [(49.12927299, -122.79221177, 10)])

    print geo.width, geo.height
    width = geo.width
    height = geo.height

# run the algorithm
    astar = AStar()

    astar.init_grid3([0, 0], [width - 1,height - 1], obstacles, geo.gridpoints,width, height)
    astar.process()
    print astar.grid_path
    astar.plot_all()



