# Astar_geo: adapting the A* code to have geographic reference

# Richard Arthurs
# Main example: https://stackoverflow.com/questions/40342355/how-can-i-generate-a-regular-geographic-grid-using-python

# generate the list of geo coords and tag with x,y
# append those or somehow bring them in to the Node object creation
# then each node object will have a coordinate
# at each waypoint, run A* search to the next waypoint
# the result list will be a path of coordinates, so fly it (potentially look ahead a few places to alter the flight vector)
#   - take average of the vector from current location to the next few in the sequence (will cut corners a bit)

#-------------------
# how this works: transform the initial coordinate into a system that uses metres to describe locations.
# Now that the reference is metres, increment those according to the step size.
# Transform those metre positions back into coordinate degrees to have a grid of coordinates

import shapely.geometry
import pyproj
import numpy as np
import dronekit as dk
from math import sqrt

class GeoCoords(object):
    def __init__(self,sw_in, ne_in, stepsize):
        # based on http://boulter.com/gps/distance/, the input stepsize is twice as large as what this procedure calculates
        # therefore, to actually retrieve the requested resolution, multiply it by 2
        # I think this is because "metres" get wider near the equator? Either way, this is accurate.
        stepsize = stepsize*2

        # Set up projections
        p_ll = pyproj.Proj(init='epsg:4326') # lat lon projection
        p_mt = pyproj.Proj(init='epsg:3857') # metric; same as EPSG:900913, in metres

        # Create corners of the rectangle that outlines the grid
        sw = shapely.geometry.Point((sw_in[1], sw_in[0])) # lon, lat
        ne = shapely.geometry.Point((ne_in[1], ne_in[0]))

        # Project corners to target projection (turn to metres)
        s = pyproj.transform(p_ll, p_mt, sw.x, sw.y) # Transform point to 3857
        e = pyproj.transform(p_ll, p_mt, ne.x, ne.y)

        # Iterate over width and height
        self.gridpoints = np.empty([0,4])
        x = s[0]
        while x < e[0]:
            y = s[1]
            while y < e[1]:
                p = pyproj.transform(p_mt, p_ll, x, y)
                d = (x-s[0],y-s[1],p[1],p[0]) # x, y, lat, lon
                self.gridpoints = np.vstack((self.gridpoints, d))
                y += stepsize
            x += stepsize

        # determine the number of divisions
        discard, self.width = np.unique(self.gridpoints[:,1], return_counts=True, axis = 0) # number of x divisions
        discard, self.height = np.unique(self.gridpoints[:,0], return_counts=True, axis = 0) # number of y divisions
        self.width = self.width[0]
        self.height = self.height[0]

    def saveCSV(self):
        # this CSV can be directly copied and pasted here: https://www.darrinward.com/lat-long/
        with open('testout.csv', 'wb') as of:
            for row in self.gridpoints:
                of.write('{:f};{:f}\n'.format(row[2], row[3]))

                # will need to pick out the obstacles. Do by turning the obs location x,y into a linear index and grabbing the point with it

    def getSpatialResolution(self):
        dlat = self.gridpoints[0,2] - self.gridpoints[1,2]
        dlong = self.gridpoints[0,3] - self.gridpoints[1,3]
        res = sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

        print 'Note: spatial resolution is approximate, and should be slightly better than the following value:'
        print 'Spatial resolution: ', res,'metres'
        return res


