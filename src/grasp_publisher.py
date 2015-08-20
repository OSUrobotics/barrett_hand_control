#!/usr/bin/python

import rospy
from wam_msgs.msg import HandCommand
from std_msgs.msg import Empty
from math import pi
import sys,getopt
import csv

class JointPub(object):
    def __init__(self,vec):
        rospy.init_node('JointPub',anonymous = True)
        self.pub = rospy.Publisher("/bhand/hand_cmd", HandCommand, queue_size=1)
        self.hand_command_template = HandCommand()
        print "working"
        self.publish_jnts(vec)

    def publish_jnts(self,joint_list):
        self.hand_command_template.f1 = joint_list[10]
        self.hand_command_template.f2 = joint_list[13]
        self.hand_command_template.f3 = joint_list[15]
        self.hand_command_template.spread = joint_list[9]
        print self.hand_command_template
        try:
            while not rospy.is_shutdown():
                self.pub.publish(self.hand_command_template)
        except rospy.exceptions.ROSInterruptException:
            print "closing node"

if __name__=="__main__":
    argv = sys.argv[1:]
    try: 
        opts, args = getopt.getopt(argv, "hc:",["icsv"])
    except getopt.GetoptError:
        print "Usage: grasp_publisher.py -c < inputfile.csv>"
        sys.exit(2)
    
    for opt,arg in opts:
        if opt=="-h":
            print "Usage: grasp_publisher.py -c <inputfile.csv>"
            sys.exit(2)
        elif opt in ("-c", "--icsv"):
            inputcsv = arg

    try: 
        filename = open(inputcsv,'rb')
    except NameError:
        print "Error: CSV file needed to visualize all grasp. Please Run grasp_generator.py before running grasp_publisher.py"
        sys.exit(2)

    vector = []
    reader = csv.reader(filename,delimiter = ',')
    for row in reader:
        row_float = []
        for x in row:
            row_float.append(float(x))
        vector.append(row_float)
            
    vals = vector[2]

    def publish_joint_state():
        global vals
        try:
            JointPub(vals)
            valid_entry = 0
            while not valid_entry:
                inp = raw_input("Do you want to try another grasp (y/n)")
                if inp == 'y':
                    valid_entry = 1
                    switch_grasp()
                elif inp == 'n':
                    valid_entry = 1
                    print "exiting"
                    sys.exit(2)
                else:
                    print "Enter a valid entry"
        except:
            pass

    def switch_grasp():
        global vals
        valid_entry = 0
        while not valid_entry:
            inp = raw_input( " 1: Highest Range of the grasp \n 2: Lowest Range of grasp \n 3: 1st new grasp \n 4: 2nd new grasp \n Enter your choice : \n")
            if inp=='1':
                vals=vector[0]
                valid_entry = 1
            elif inp=='2':
                vals = vector[3]
                valid_entry = 1
            elif inp=='3':
                vals = vector[1]
                valid_entry =1
            elif inp =='4':
                vals = vector[2]
                valid_entry=1
            else:
                print "please enter a valid entry from 1-4: "

            if valid_entry==1:
                publish_joint_state()
    switch_grasp()

