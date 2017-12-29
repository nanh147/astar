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

# Set up projections
p_ll = pyproj.Proj(init='epsg:4326')
p_mt = pyproj.Proj(init='epsg:3857') # metric; same as EPSG:900913

# Create corners of rectangle to be transformed to a grid
sw = shapely.geometry.Point((-122.796805,49.128397))
ne = shapely.geometry.Point((-122.790330,49.129779))

stepsize = 50 # note: this is not metres in Surrey (due to projection widening near the equator). Metres = stepsize/2 ish
# using haversine, come up with a better way to understand the grid resolution

# Project corners to target projection
s = pyproj.transform(p_ll, p_mt, sw.x, sw.y) # Transform point to 3857
e = pyproj.transform(p_ll, p_mt, ne.x, ne.y) #

# Iterate over 2D area
gridpoints = np.empty([0,4])
x = s[0]
while x < e[0]:
    y = s[1]
    while y < e[1]:
        p = pyproj.transform(p_mt, p_ll, x, y)
        d = (x-s[0],y-s[1],p[0], p[1])
        gridpoints = np.vstack((gridpoints, d))
        y += stepsize
    x += stepsize

discard, numx = np.unique(gridpoints[:,0], return_counts=True, axis = 0) # number of x divisions
discard, numy = np.unique(gridpoints[:,1], return_counts=True, axis = 0) # number of y divisions

print 'Generation complete.'
# this CSV can be directly copied and pasted here: https://www.darrinward.com/lat-long/
with open('testout.csv', 'wb') as of:
    for row in gridpoints:
        of.write('{:f};{:f}\n'.format(row[3], row[2]))