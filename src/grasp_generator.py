from openravepy import *
import sys,getopt
import numpy as np
import csv

env = Environment()
argv = sys.argv[1:]
try:
    opts, args = getopt.getopt(argv, "hi:o:", ["infile=","outfile="])
except getopt.GetoptError:
    print 'Usage: grasp_generator.py -i <inputfile.dae> -o <outputfile.csv>'
    sys.exit(2)
for opt,arg in opts:
    if opt == '-h':
        print 'Usage: grasp_generator.py -i <inputfile.dae> -o <outputfile.csv>'
        sys.exit(2)
    elif opt in ("-i","--infile"):
        inputfile = arg
    elif opt in ("-o","--outfile"):
        outputfile = arg

env.Load(inputfile)
robots = env.GetRobots()
Bodies = env.GetBodies()
obj = Bodies[-2]

robots_joint_angles = [robots[0].GetDOFValues(),robots[1].GetDOFValues(),robots[2].GetDOFValues()]

length_of_jnt_vector = [len(robots[0].GetDOFValues()), len(robots[1].GetDOFValues()), len(robots[2].GetDOFValues())]
main_robot_joint_angles = np.array(robots_joint_angles[length_of_jnt_vector.index(17)])
dummy_hand_joint_angles = np.array(robots_joint_angles[length_of_jnt_vector.index(7)])
dummy_wam_joint_angles =  np.array(robots_joint_angles[length_of_jnt_vector.index(10)])

dummy_robot_joint_angles = np.append(dummy_hand_joint_angles,dummy_wam_joint_angles)

difference_vector = []

for i in range(len(main_robot_joint_angles)):
    difference_vector.append(main_robot_joint_angles[i]-dummy_robot_joint_angles[i])

new_grasp_1 = []
new_grasp_2 = []

for i in range(len(main_robot_joint_angles)):
    new_grasp_1.append(dummy_robot_joint_angles[i]+0.3*difference_vector[i])
    new_grasp_2.append(dummy_robot_joint_angles[i]+0.6*difference_vector[i])
main_robot_joint_angles = main_robot_joint_angles.tolist()
dummy_robot_joint_angles = dummy_robot_joint_angles.tolist()
#print main_robot_joint_angles
#print dummy_robot_joint_angles
#print new_grasp_1
#print new_grasp_2
try:
    filename = open(outputfile, 'wb')
    writer = csv.writer(filename)
    writer.writerows([main_robot_joint_angles, dummy_robot_joint_angles, new_grasp_1, new_grasp_2])
except NameError: 
    print 'Usage: grasp_generator.py -i <inputfile.dae> -o <outputfile.csv>'
