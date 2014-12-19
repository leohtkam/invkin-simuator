from math import pi, cos, sin

# Return a list of points representing a circle with radius R, centered at origin and on xz plane.
# With DIVISION points.
# The first point is at (0, 0, R) and go counter-clockwise.
def getCircle(r, division=1800):
    p = []
    step = 2 * pi / division
    for i in xrange(division):
        p.append([r * sin(i * step), 0.0, r * cos(i * step)])
    return p

def getShape(r, x_factor, z_factor, division=1800):
    p = []
    step = 2 * pi / division
    for i in xrange(division):
        x = r * sin(i * step)
        z = r * cos(i * step)
        y = x * x_factor + (z - r) * z_factor
        p.append([x, y, z])
    return p

def writeToFile(filename, points):
    f = open(filename, "w")
    for x, y, z in points:
        f.write("%f %f %f\n" % (x, y, z))
    f.close()