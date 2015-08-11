#!/usr/bin/env python
import threading
from openravepy import *
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import sys
import rospkg
import time


class control(object):
    def __init__(self):
        rospy.init_node('control',anonymous = True)
        self.path = rospkg.RosPack().get_path('barrett_hand_control')
	self.env = None
        self.flag = 0

    def generate_environment(self):
        try:
            self.env = Environment()
            self.env.Load(self.path+'/src/barrett_wam.dae')
            self.robot = self.env.GetRobots()[0]
            self.hand = self.env.ReadRobotXMLFile(self.path+'/src/bhand.dae')
            self.env.Add(self.hand)
            self.obj = self.env.ReadKinBodyXMLFile(self.path+'/src/stl_files/Coil1.STL',{'scalegeometry':'0.001 0.001 0.001'})
            self.env.Add(self.obj)
            self.env.SetViewer('qtcoin')
            self.Trans_matrix = np.array([[1,0,0,0],[0,1,0,-0.5],[0,0,1,1.5],[0,0,0,1]])
            self.obj.SetTransform(self.Trans_matrix)
            self.T_robot = self.robot.GetLinkTransformations()[9:23]
            self.hand.SetLinkTransformations(self.T_robot)
            while not rospy.is_shutdown():
                    N=1

        except KeyboardInterrupt, e:
            print e
        finally:
            print 'exiting'
            self.env.Destroy()

    def updater(self,slider_values):
        self.flag = slider_values.data[17]
        if self.flag == 1:
            self.T_robot = self.robot.GetLinkTransformations()[9:23]
            self.hand.SetLinkTransformations(self.T_robot)
        self.robot.SetDOFValues(slider_values.data[0:17])


def main():
    ctrl = control()
    ctrl.sub = rospy.Subscriber("control_slider_values", Float32MultiArray, ctrl.updater)
    ctrl.generate_environment()

if __name__=="__main__":
    main()
