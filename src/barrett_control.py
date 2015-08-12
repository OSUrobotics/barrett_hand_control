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
            self.robot.SetDOFValues([0.7942003011703491, -1.9751428365707397, 1.642993688583374, 1.3289387226104736, -1.3093395233154297, 0.20392456650733948, 0.12288285791873932, 0.0, 0.0, 0.02515728399157524, 1.2185022830963135, 0.4061265289783478, 0.02515728399157524, 1.1969958543777466, 0.39948949217796326, 1.0065982341766357, 0.5453813076019287])
            self.env.Add(self.hand)
            self.obj = self.env.ReadKinBodyXMLFile(self.path+'/src/stl_files/PitcherBottom.STL',{'scalegeometry':'0.001 0.001 0.001'})
            self.env.Add(self.obj)
            self.Table = self.env.ReadKinBodyXMLFile('data/table.kinbody.xml')
            self.env.Add(self.Table)
            self.env.SetViewer('qtcoin')
            self.Trans_matrix = np.array([[0.0, 3.140000104904175, -3.140000104904175, -0.71670001745224],[ 0.0, 0.0, -0.8199999928474426, -0.11140000075101852], [0.0, 0.0, 0.0, 1.0211999416351318], [0.0, 0.0, 0.0, 0.0]])
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
        transform = [[slider_values.data[0],slider_values.data[1],slider_values.data[2],slider_values.data[3]],[slider_values.data[4],slider_values.data[5],slider_values.data[6],slider_values.data[7]],[slider_values.data[8],slider_values.data[9],slider_values.data[10],slider_values.data[11]],[slider_values.data[12],slider_values.data[13],slider_values.data[14],slider_values.data[15]]]
        self.Table.SetTransform(transform)


def main():
    ctrl = control()
    ctrl.sub = rospy.Subscriber("control_slider_values", Float32MultiArray, ctrl.updater)
    ctrl.generate_environment()

if __name__=="__main__":
    main()
