from math import pi, cos, sin
import numpy

# Return a list of points representing a circle with radius R, centered at origin and on xz plane.
# With DIVISION points.
# The first point is at (0, 0, R) and go counter-clockwise.
def getCircle(r, division=1800):
    p = []
    step = 2 * pi / division
    for i in xrange(division):
        p.append([r * sin(i * step), 0.0, r * cos(i * step)])
    return p

def getThing(r1, r2, r3, division=1800):
    p = []
    step = 2 * pi / division
    for i in xrange(division):
        p.append([r1 * sin(i * step) + 40, 0, r3 * cos(i * step)])
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

c = numpy.array([0., 0., 0.]);
def getCircle(u, v, r, c, division=3600):
    p = []
    step = 2 * pi / division
    for i in xrange(division):
        p.append(c + r * cos(i * step) * u + r * sin(i * step) * v)
    return p

t = 0.
u = numpy.array([0., -sin(t), cos(t)])
v = numpy.array([1., 0., 0.])
r = 50.
#exp_c = numpy.array([0., -20 * sin(t), 15 + 20 * cos(t)])
exp_c = numpy.array([0., 0., 60])
c = exp_c - r * u

p = getThing(40,0,20)
writeToFile("config/ellipse.path", p)
