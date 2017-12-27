import numpy as np
import matplotlib.pyplot as plt
import numpy.matlib

# uses midpoint circle algorithm: https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
# Richard Arthurs port from C > Python, with addition of circle filler

def gencircle(radius, x0, y0):
    x = radius - 1
    y = 0
    dx = 1
    dy = 1

    err = dx - 2*radius
    c = []

    while x >= y:
        c.append([x0 + x, y0 + y])
        c.append([x0 + y, y0 + x])
        c.append([x0 - y, y0 + x])
        c.append([x0 - x, y0 + y])
        c.append([x0 - x, y0 - y])
        c.append([x0 - y, y0 - x])
        c.append([x0 + y, y0 - x])
        c.append([x0 + x, y0 - y])

        if err <= 0:
            y = y + 1
            err = err + dy
            dy = dy + 2

        else:
            x = x - 1
            dx = dx + 2
            err = err + dx - 2*radius

    c = np.asarray(c) # convert to numpy
    uniques = np.vstack({tuple(row) for row in c}) # the unique points
    extrapts = ([0,0]) # to contain

# fill in the circles
    for row in uniques: # find the max and min y values for each x. Create the intermediate y's at each x to fill in the circle
        miny = c[c[:,0] == row[0], 1].min() # logical indexing
        maxy = c[c[:,0] == row[0], 1].max()

        tempy = np.arange(miny, maxy+1) # range of y's to make the fill
        tempx = np.matlib.repmat(row[0],np.size(tempy,0),1) # the x indices

        # horzcat the tempx and tempy, then stack that onto existing final
        extrapts = np.vstack((extrapts,np.column_stack((tempx, tempy)))) # https://stackoverflow.com/questions/14741061/concatenating-column-vectors-using-numpy-arrays

    extrapts = np.delete(extrapts,0,axis = 0) # delete the initial row of zeros used in construction
    return np.vstack((c,extrapts)) # stack  fill in points onto the original outline points


if __name__ == '__main__':
    c = gencircle(10,50,50)

    # plot the circle
    c = tuple(map(tuple, c))
    xobs, yobs = zip(*c)
    plt.plot(xobs, yobs, 'kx')
    plt.axis([0, 100, 0, 100])
    plt.show()

    print c

