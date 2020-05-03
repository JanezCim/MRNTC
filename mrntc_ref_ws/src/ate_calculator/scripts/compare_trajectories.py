# Compares two trajectories (set of points given as x \t y \t yaq \t timestamp)
# It takes points with closes timestamp to eachothers and calculates ATE between them

import numpy as np
import matplotlib.pyplot as plt
import csv, sys, math
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm

################################################PARAMETERS##############################################
see_detailed_debug = False
time_diff_warning_thresh = 0.2
save_img = True #will save the image instead of showing it, it will save under the 2nd argument name
c_range = 0.1 #(meters) calculating how many % of test points are inside this range


########################################################################################################

if __name__ == "__main__":
  # extracts points from a file: x,y separated by tab
  if (len(sys.argv)!=3 and len(sys.argv)!=4):
    print "This script accepts 2 or 3 arguments, but " + str(len(sys.argv)-1) + " are given."
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
  
  x1 = []
  y1 = []
  times1 = []
  with open(sys.argv[2]) as tsv:
    for line in csv.reader(tsv, dialect="excel-tab"):
      fp = float(line[0])
      if(not math.isnan(fp)):
        x1.append(float(line[0]))
        y1.append(float(line[1]))
        times1.append(float(line[3]))

  #checks if the given list was empty or wrong
  if(len(ref_x)==0):
    print "Imported coordinate list empty or wrong format"
    exit()

  ate_list1 = []
  exclueded_count = 0
  for i in range(0,len(ref_x)):
    min_time_dif = sys.float_info.max
    min_time_j = 0
    for j in range(0,len(x1)):
      d = abs(times1[j]-ref_times[i]) 
      if(d<min_time_dif):
        min_time_dif = d
        min_time_j = j
    
    if(min_time_dif>time_diff_warning_thresh):
      if(see_detailed_debug):
        print "Warning: time diff between points its too large, it is not counted into ATE calculation. The min time difference between ref point "+ str(i) +" and red point " + str(min_time_j)+ " is " + str(min_time_dif)
      exclueded_count = exclueded_count+1
      continue

    dx = ref_x[i]-x1[min_time_j]
    dy = ref_y[i]-y1[min_time_j]

    ate_list1.append(math.sqrt(dx*dx+dy*dy))

  green_ate = sum(ate_list1)/len(ate_list1)

  print "First test trajectory (green line) ATE: " + str(green_ate)
  print str(exclueded_count) + " points were excluded because of large time difference, enable see_detailed_debug to see why"

  blue_ate = 0
  ate_list2 = []
  if(len(sys.argv)==4):
    x2 = []
    y2 = []
    times2 = []
    with open(sys.argv[3]) as tsv:
      for line in csv.reader(tsv, dialect="excel-tab"):
        fp = float(line[0])
        if(not math.isnan(fp)):
          x2.append(float(line[0]))
          y2.append(float(line[1]))
          times2.append(float(line[3]))

    #checks if the given list was empty or wrong
    if(len(x2)==0):
      print "Imported coordinate list empty or wrong format"
      exit()

    
    exclueded_count = 0
    for i in range(0,len(ref_x)):
      min_time_dif = sys.float_info.max
      min_time_j = 0
      for j in range(0,len(x2)):
        d = abs(times2[j]-ref_times[i]) 
        if(d<min_time_dif):
          min_time_dif = d
          min_time_j = j
      
      if(min_time_dif>time_diff_warning_thresh):
        if(see_detailed_debug):
          print "Warning: time diff between points its too large, it is not counted into ATE calculation. The min time difference between ref point "+ str(i) +" and blue point " + str(min_time_j)+ " is " + str(min_time_dif)
        exclueded_count = exclueded_count+1
        continue

      dx = ref_x[i]-x2[min_time_j]
      dy = ref_y[i]-y2[min_time_j]

      # calculation of ate
      ate_list2.append(math.sqrt(dx*dx+dy*dy))

    blue_ate = sum(ate_list2)/len(ate_list2)
    print "Second test trajectory (blue line) ATE: " + str(blue_ate)
    print str(exclueded_count) + " points were excluded because of large time difference, enable see_detailed_debug to see why"

  # check if the ate list is empty
  if(len(ate_list1)==0):
    print "ATE list is empty, cannot calucalte statistics"
    exit()

  ######### calculating green RMSE (root mean square error)
  rmse = 0
  for ate in ate_list1:
    rmse = rmse + ate*ate
  rmse = math.sqrt(rmse/len(ate_list1))
  #########################################################

  ######## calculating how many % of points are inside a range
  inside_range_count = 0
  inside_range_percent = 0
  for ate in ate_list1:
    if(ate<c_range):
      inside_range_count = inside_range_count+1

  inside_range_percent = inside_range_count/float(len(ate_list1))
  ############################################################

  # define the size of the figure
  plt.figure(figsize=(12,12))

  # create a figure title
  plt.title("Ref:"+str(sys.argv[1])+
            "\nTest:"+str(sys.argv[2])+
            "\nAvarage ATE:"+str(green_ate)+
            "\nMax ATE: "+str(max(ate_list1))+
            "\nRMSE: "+str(rmse)+
            "\n"+str(inside_range_percent*100)+" percent of test points is within radius "+str(c_range)+"m of reference"+
            "\n")
  
  # plot the red reference trajectory
  plt.plot(ref_x, ref_y, "-ro", label="Reference trajectory")
  
  # plot the first test trajectory
  plt.plot(x1, y1, "-go", label="Test trajectory, Avg. ATE:"+str(green_ate))
  
  # if there was a 3rd argument given, plot the second test trajectory
  if(len(sys.argv)==4):
    plt.plot(x2, y2, "-bo", label="Test trajectory, Avg. ATE:"+str(blue_ate))
  
  # label the coordinates
  plt.xlabel("X coordinate (m)")
  plt.ylabel("Y coordinate (m)")

  # location of the legend
  plt.legend(loc="upper left")

  # equalise the distances on axes
  plt.axis('scaled')
  
  # to save the image or to show it
  if(save_img):
    plt.savefig(str(sys.argv[2])+'.png')
    print "SAVING IMAGES INSTEAD OF SHOWING THEM"
  else:
    plt.show()