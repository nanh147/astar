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
# Transform those metre positions into coordinate degrees

import shapely.geometry
import pyproj
import numpy as np

# Set up projections
p_ll = pyproj.Proj(init='epsg:4326')
p_mt = pyproj.Proj(init='epsg:3857') # metric; same as EPSG:900913

# Create corners of rectangle to be transformed to a grid
# SW: 49.12836,-122.7972
# NE: 49.12934, -122.7900

nw = shapely.geometry.Point((-122.7953373,49.1289449))
se = shapely.geometry.Point((-122.7911861,49.1283074))

stepsize = 5 # 5 m grid step size

# Project corners to target projection
s = pyproj.transform(p_ll, p_mt, nw.x, nw.y) # Transform NW point to 3857
e = pyproj.transform(p_ll, p_mt, se.x, se.y) # .. same for SE

# Iterate over 2D area
gridpoints = np.empty([1,4])
x = s[0]
while x < e[0]:
    print 'xitr'
    y = s[1]
    while y > e[1]:

        # p = shapely.geometry.Point(pyproj.transform(p_mt, p_ll, x, y))
        p = pyproj.transform(p_mt, p_ll, x, y)
        d = (p[0], p[1], x-s[0],y-s[1])
        gridpoints = np.vstack((gridpoints, d))
        # gridpoints.append(p)
        y -= stepsize
    x += stepsize

    # gridpoints = np.unique(gridpoints, axis = 0)

with open('testout.csv', 'wb') as of:
    of.write('lon;lat\n')
    for p in gridpoints:
        of.write('{:f};{:f}\n'.format(p.x, p.y))