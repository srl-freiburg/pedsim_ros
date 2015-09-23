#!/usr/bin/env python
import png, rospkg, numpy
import xml.etree.ElementTree as xml

filename = "social_contexts.xml"

path = ""
if not "/" in filename:
    rospack = rospkg.RosPack()
    path = rospack.get_path("pedsim_simulator") + "/scenarios/"

filepath = path + filename
print "Reading file " + filepath

tree = xml.parse(filepath)
root = tree.getroot()

xmin = 9999999999
ymin = 9999999999
xmax = 0
ymax = 0

obstacles = []
for obstacle in root.findall("obstacle"):
    xlimits = int(obstacle.get("x1")), int(obstacle.get("x2"))
    ylimits = int(obstacle.get("y1")), int(obstacle.get("y2"))
    obstacle = ( min(xlimits), min(ylimits), max(xlimits), max(ylimits))
    xmin = min(xmin, obstacle[0])
    ymin = min(ymin, obstacle[1])
    xmax = max(xmax, obstacle[2])
    ymax = max(ymax, obstacle[3])
    obstacles.append(obstacle)

print "Map dimensions: (%d, %d) -- (%d, %d)" % (xmin, ymin, xmax, ymax)
assert(xmin >= 0)
assert(ymin >= 0)
width = xmax + 1
height = ymax + 1

grid = numpy.uint8( numpy.zeros( (height, width) ) )

for obstacle in obstacles:
    for x in xrange(obstacle[0], obstacle[2] + 1):
        for y in xrange(obstacle[1], obstacle[3] + 1):
            grid[x, y] = 255

outputFilename = filename
if ".xml" in outputFilename:
    outputFilename = outputFilename.replace(".xml", ".png")
else:
    outputFilename += ".png"

print "Writing output to " + outputFilename
f = open(outputFilename, 'wb')      # binary mode is important
w = png.Writer(width, height, greyscale=True)
w.write(f, grid)
f.close()