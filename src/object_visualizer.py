#!/usr/bin/env python
import threading
from openravepy import *
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import sys
import rospkg
import time


class object_visualizer(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path('barrett_hand_control')
        self.env = Environment()
        self.env.Load(self.path+'/src/bhand.dae')
        self.hand_1 = self.env.GetRobots()[0]
        self.hand_2 = self.env.ReadRobotXMLFile(self.path+'/src/new_bhand.dae')
        self.obj = self.env.ReadKinBodyXMLFile(self.path+'/src/stl_files/CerealBox.STL',{'scalegeometry':'0.001 0.001 0.001'})
        self.env.Add(self.obj)
        self.env.Add(self.hand_2)
        self.env.SetViewer('qtcoin')
        self.obj.SetTransform(np.eye(4))
        self.hand_1_curr_mat = self.hand_1.GetLinkTransformations()
        self.hand_2_curr_mat = self.hand_2.GetLinkTransformations()
        self.obj_curr_mat = self.obj.GetTransform()
        self.flag = False
    
    def Transformation_switcher(self,transform_1,transform_2):
        return np.dot(np.linalg.inv(transform_1),transform_2) 

    def hand_transformer(self,hand_mat):
        self.hand_1_curr_mat = hand_mat.data[0:-1]
        if hand_mat.data[-1]==1:
            self.flag = True
        else:
            self.flag = False
        self.hand_1_curr_mat = np.array(self.hand_1_curr_mat)
        self.hand_1_curr_mat = self.hand_1_curr_mat.reshape(14,4,4)
    
    def part_transformer(self,part_mat):
        self.obj_curr_mat = part_mat.data
        self.obj_curr_mat = np.array(self.obj_curr_mat)
        self.obj_curr_mat = self.obj_curr_mat.reshape(4,4)
        self.obj.SetTransform(self.obj_curr_mat)

    def generate_environment(self):
        try:
            while not rospy.is_shutdown():
                hand_wrt_obj = self.Transformation_switcher(self.obj.GetTransform(),self.hand_1_curr_mat[0])
                print "hand",len(self.hand_1_curr_mat)
                print "obj",len(self.hand_1.GetLinkTransformations())
                self.hand_1.SetLinkTransformations(self.hand_1_curr_mat)
                #self.hand_1.SetTransform(self.hand_1_curr_mat[0])
                if self.flag==True:
                    self.hand_2.SetLinkTransformations(self.hand_2_curr_mat)
                    #self.hand_2.SetTransform(hand_wrt_obj)
                else:
                    self.hand_2.SetLinkTransformations(self.hand_2_curr_mat)
                    #self.hand_2.SetTransform(hand_wrt_obj)

        except KeyboardInterrupt:
            print "exiting"


def main():
    ctrl = object_visualizer()
    rospy.init_node('object_visualizer',anonymous = True)
    ctrl.sub = rospy.Subscriber("/hand_transformation", Float32MultiArray, ctrl.hand_transformer)
    ctrl.sub_trans = rospy.Subscriber("/obj_transformation",Float32MultiArray, ctrl.part_transformer)
    ctrl.generate_environment()

if __name__=="__main__":
    main()
