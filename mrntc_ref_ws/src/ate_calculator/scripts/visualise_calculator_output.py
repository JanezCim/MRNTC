# Imports list of ATEs and timestamps separated by tab (ate \t timestamp)
# and prints out the avarage overall ATE as well as some other stuff.
# ATE: absolute trajectory error

import csv
import matplotlib.pyplot as plt
import sys
import math

data = []
time = []

if (len(sys.argv)!=2):
  print "This script accepts 1 argument, but " + str(len(sys.argv)-1) + " are given."
  exit()

summ = 0 

with open(sys.argv[1]) as tsv:
    for line in csv.reader(tsv, dialect="excel-tab"):
      fp = float(line[0])
      if(not math.isnan(fp)):
        data.append(float(line[0]))
        time.append(float(line[1]))

if(len(data)<2):
  print "There are no entries in this folder"
  exit()

print "Num of datapoints: " + str(len(data))
print "Avgarage num of datapoints per second:  " + str(len(data)/(time[len(time)-1]-time[0]))
print "Avarage:" + str(sum(data)/len(data))

plt.plot(time, data)
plt.xlabel('Time (ms)')
plt.ylabel(sys.argv[1])
plt.show()