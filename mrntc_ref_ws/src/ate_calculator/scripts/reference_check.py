# Compares the coordinate set of coordinates (given as x \t y) from a file (as argument) to a polygon.
# polygon is given as a set of four points

import numpy as np
import matplotlib.pyplot as plt
import csv, sys, math
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm


# input polygon points
polygon = [(0,0), (1.0,0), (1.0,-0.583), (0,-0.583)]

if __name__ == "__main__":
  rx = []
  ry = []
  for point in polygon:
    rx.append(point[0])
    ry.append(point[1])

  # extracts points from a file: x,y separated by tab
  if (len(sys.argv)!=2):
    print "This script accepts 1 argument, but " + str(len(sys.argv)-1) + " are given."
    exit()

  cx = []
  cy = []

  with open(sys.argv[1]) as tsv:
      for line in csv.reader(tsv, dialect="excel-tab"):
        fp = float(line[0])
        if(not math.isnan(fp)):
          cx.append(float(line[0]))
          cy.append(float(line[1]))

  #checks if the given list was empty or wrong
  if(len(cx)==0):
    print "Imported coordinate list empty or wrong format"
    exit()

  # calculation of ate

  ### Algorithm breefly:
  ### Measuring the distance of p3 (each coordinate from list)
  ### from line formed by p1 and p2 (each line from polygon).
  ### The shortest distance of each p3 gets summed up and avaraged at the end.

  summ = 0
  max_offset = 0
  max_offset_index = 0  
  # with p3 go through all the path points
  for i in range(0,len(cx)):

    min_d = sys.float_info.max
    p3 = np.asarray((cx[i],cy[i]))
    # with p1 and p2 go throuhg all lines of the polygon but the last one
    for j in range(0,len(polygon)-1):
      p1 = np.asarray(polygon[j])
      p2 = np.asarray(polygon[j+1])
      d = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
      
      if(d<min_d):
        min_d = d
    
    #to connect the first and the last point in polygon (last line)
    p1 = np.asarray(polygon[len(polygon)-1])
    p2 = np.asarray(polygon[0])
    d = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
    if(d<min_d):
      min_d = d
      
    summ = summ + min_d

    #seach for the biggest offset
    if(min_d>max_offset):
      max_offset = min_d
      max_offset_index = i

  print "ATE: " + str(summ/len(cx))

  # define the size of the figure
  plt.figure(figsize=(12,8))

  plt.title("Plot source: "+str(sys.argv[1])+
            "\nATE: "+str(summ/len(cx))+"m"+
            "\nMax offset: "+str(max_offset)+"m")
  plt.plot(rx, ry, "-ro", label="Ground truth")
  plt.plot(cx, cy, label="Reference trajectory")
  plt.plot(cx[max_offset_index], cy[max_offset_index], 'ro')
  plt.ylabel("Y coordinate (m)")
  plt.xlabel("X coordinate (m)")
  plt.legend(loc="upper left")
  plt.ylim(-3, 1.5)
  plt.axis('scaled')
  plt.show()