# Compares two trajectories (set of points given as x \t y \t yaq \t timestamp)
# It takes points with closes timestamp to eachothers and calculates ATE between them

import numpy as np
import matplotlib.pyplot as plt
import csv, sys, math
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm

time_diff_warning_thresh = 0.2

if __name__ == "__main__":
  # extracts points from a file: x,y separated by tab
  if (len(sys.argv)!=3):
    print "This script accepts 2 arguments, but " + str(len(sys.argv)-1) + " are given."
    exit()

  ref_x = []
  ref_y = []
  ref_times = []

  with open(sys.argv[1]) as tsv:
      for line in csv.reader(tsv, dialect="excel-tab"):
        fp = float(line[0])
        if(not math.isnan(fp)):
          ref_x.append(float(line[0]))
          ref_y.append(float(line[1]))
          ref_times.append(float(line[3]))
  
  x = []
  y = []
  times = []
  with open(sys.argv[2]) as tsv:
      for line in csv.reader(tsv, dialect="excel-tab"):
        fp = float(line[0])
        if(not math.isnan(fp)):
          x.append(float(line[0]))
          y.append(float(line[1]))
          times.append(float(line[3]))

  #checks if the given list was empty or wrong
  if(len(ref_x)==0):
    print "Imported coordinate list empty or wrong format"
    exit()

  ate_list = []

  for i in range(0,len(ref_x)):
    min_time_dif = sys.float_info.max
    min_time_j = 0
    for j in range(0,len(x)):
      d = abs(times[j]-ref_times[i]) 
      if(d<min_time_dif):
        min_time_dif = d
        min_time_j = j
    
    if(min_time_dif>time_diff_warning_thresh):
      print "Time difference of the point " + str(i)+ " is " + str(min_time_dif)

    dx = ref_x[i]-x[min_time_j]
    dy = ref_y[i]-y[min_time_j]

    ate_list.append(math.sqrt(dx*dx+dy*dy))

  print "avarage ATE: " + str(sum(ate_list)/len(ate_list))

  plt.plot(ref_x, ref_y, "-go")
  plt.plot(x, y, "r")
  plt.axis('scaled')
  plt.show()