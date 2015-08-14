#!/usr/bin/python

from openravepy import *
import sys,getopt
import csv
import time

argv = sys.argv[1:]

try:
    opts, args = getopt.getopt(argv, "hd:c:", ["idae","icsv"])
except getopt.GetoptError:
    print "Usage: grasp_visualizer.py -d <inputfile.dae> -c <inputfile.csv>"
    sys.exit(2)

for opt, arg in opts:
    if opt=="-h":
        print "Usage: grasp_visualizer.py -i <inputfile.dae> -i <inputfile.csv>"
        sys.exit(2)
    elif opt in ("-d","--idae"):
        inputdae = arg
    elif opt in ("-c","--icsv"):
        inputcsv = arg

try:
    filename = open(inputcsv, 'rb')
except NameError:
    print "Error: CSV file needed to visualize all grasp. Please Run grasp_generator.py before running grasp_visualizer.py"
    sys.exit(2)

vector = []
reader = csv.reader(filename, delimiter = ',')
for row in reader:
    row_float = []
    for x in row:
        row_float.append(float(x))
    vector.append(row_float)

try:
    env = Environment()
    env.Load(inputdae)
except NameError:
    print "Error: .dae file needed to generate the environment to visualize the grasp"
    sys.exit(2)

robot = env.GetRobots()[0]
env.SetViewer('qtcoin')
robot_1 = env.GetRobots()[1]
robot_1.SetVisible(0)
robot_2 = env.GetRobots()[2]
robot_2.SetVisible(0)

try:
    while True:
        robot.SetDOFValues(vector[0])
        time.sleep(1)
        robot.SetDOFValues(vector[1])
        time.sleep(1)
        robot.SetDOFValues(vector[2])
        time.sleep(1)
        robot.SetDOFValues(vector[3])
        time.sleep(1)
except KeyboardInterrupt:
    print "exiting"



