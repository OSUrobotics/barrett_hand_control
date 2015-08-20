#!/usr/bin/python

import rospy
from wam_msgs.msg import HandCommand
from std_msgs.msg import Empty
from math import pi
import sys,getopt
import csv
import threading

class JointPub(object):
    def __init__(self):
        rospy.init_node('JointPub',anonymous = True,disable_signals = True)
        self.pub = rospy.Publisher("/bhand/hand_cmd", HandCommand, queue_size=1)
        self.hand_command_template = HandCommand()

    def publish_jnts(self,joint_list):
        self.hand_command_template.f1 = joint_list[10]
        self.hand_command_template.f2 = joint_list[13]
        self.hand_command_template.f3 = joint_list[15]
        self.hand_command_template.spread = joint_list[9]
        self.pub.publish(self.hand_command_template)

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
    jnts_obj = JointPub()
    def publish_joint_state():
        global vals
        try:
            while not rospy.is_shutdown():
                jnts_obj.publish_jnts(vals)
        except rospy.ROSException:
            rospy.signal_shutdown("program Killed")
            print "\n"

    def switch_grasp():
        global vals
        try:
            while not rospy.is_shutdown():
                 inp = raw_input( " \n\n 1: Highest Range of the grasp \n 2: Lowest Range of grasp \n 3: 1st new grasp \n 4: 2nd new grasp \n \n      OR        \n \n<Ctrl-D> to kill program \n \n Enter your choice : ")
                 if inp=='1':
                     vals=vector[0]
                 elif inp=='2':
                     vals = vector[3]
                 elif inp=='3':
                     vals = vector[1]
                 elif inp =='4':
                     vals = vector[2]
                 else:
                     print "please enter a valid entry from 1-4: "
        except EOFError:
            print "\n"
            rospy.signal_shutdown("program killed")

    t_1 = threading.Thread(name='switch_grasp', target=switch_grasp)
    t_2 = threading.Thread(name='publish_joint_state', target = publish_joint_state)
    t_1.start()
    t_2.start()

