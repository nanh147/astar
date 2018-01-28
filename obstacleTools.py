import numpy as np
from gencircle import *
from GeoCoords import *
# Obstacle tools - functions related to obstacle generation
# Author: Richard Arthurs


## ----------------------- FOR USE WITH INTEROP ----------------------------------------------------------------------------------

def genObstaclesGeo(geo, obstacles_input):
    # obstacles_input: LAT, LON, RADIUS
    # To be used for generating obstacles from external data (interop)
    obs_points = np.zeros([1, 2]) #using empty was placing a coord pair in the first row -- internal issue? Makes no sense

    for row in obstacles_input:
        print row
        xy_pos = geo.getClosestPoint((row[0], row[1]))
        obs_points = np.vstack((obs_points, gencircle(row[2], xy_pos[0], xy_pos[1]))) # pass the xy since that's how circles are generated

    obs_points = obs_points[1:, :] # Remove 0th row since it's zeros
    return tuple(map(tuple, obs_points))  # convert to tuple



## ----------------------- TESTS ----------------------------------------------------------------------------------

def genTestObstacles(num_obstacles, width, height, circle = 1):
    # randomly distributed point obstacles test. Random xy, not geographic placement
    if not circle:
        obstacles = np.round(np.random.rand(num_obstacles,2) * [width, height])
        obstacles = checkObstacles(obstacles, width, height)
        return tuple(map(tuple, obstacles))

    # randomly placed circular objects
    else:
        obstacle_locs = np.round(np.random.rand(num_obstacles, 2) * int(np.mean([width, height]))) # scale by mean so that rectangular grids don't chop most objects
        obstacles = np.empty([1, 2])
        for row in obstacle_locs:
            obstacles = np.vstack((obstacles, gencircle(10, row[0], row[1])))

        obstacles = checkObstacles(obstacles, width, height)
        return tuple(map(tuple, obstacles)) # convert to tuple

def genTestObstaclesGeo(num_obstacles, geo, radius):
    # Generate obstacles via coordinate location (as we'll receive from interop"
    obs_rand_weights = np.random.rand(num_obstacles, 2) # generate random weights for lat, lon. Multiply flight range by these weights to get the random locations

    # linearly interpolate based on the random weighting
    # note: this will probably break in other hemispheres due to signs assumed in interpolation
    obstacle_centre_coords = ([np.min([geo.sw_lat,geo.ne_lat]),np.min([geo.sw_lon,geo.ne_lon])] + [np.abs(geo.sw_lat - geo.ne_lat),np.abs(geo.sw_lon - geo.ne_lon)] * obs_rand_weights)
    indices = geo.getClosestPoint(obstacle_centre_coords) # get the closest location gridpoints row
    obs_points = np.zeros([1, 2]) #using empty was placing a coord pair in the first row -- internal issue? Makes no sense

    for row in indices:
        obs_points = np.vstack((obs_points, gencircle(radius, row[0], row[1]))) # pass the xy since that's how circles are generated

    obs_points = obs_points[1:, :] # Remove 0th row since it's zeros
    return tuple(map(tuple, obs_points))  # convert to tuple

    # also reduce the lat,lon range by x% so we have fewer cut off obstacles

## ----------------------- UTILITIES----------------------------------------------------------------------------------


def checkObstacles(obstacles, width, height):
    # the obstacle matrix must be unique integers that are within the map. This function will
    # strip out obstacles that are out of range
    obstacles = obstacles.astype(int)
    obstacles = obstacles[obstacles[:, 0] >= 0]  # x GTE 0
    obstacles = obstacles[obstacles[:, 0] < width]  # x LT width
    obstacles = obstacles[obstacles[:, 1] >= 0]  # y GTE 0
    obstacles = obstacles[obstacles[:, 1] < height]  # y LT height

    obstacles = np.unique(obstacles,
                          axis=0)  # must remove duplicate obstacles too or they'll be added to the list of locations :(
    return obstacles
