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
        self.env.Load(self.path+'/src/stl_files/CerealBox.STL',{'scalegeometry':'0.001 0.001 0.001'})
        self.env.SetViewer('qtcoin')
        self.obj = self.env.GetBodies()[0]
        error_1 = self.env.Load(self.path+'/src/bhand.dae')
        error_2 = self.env.Load(self.path+'/src/new_bhand.dae')
        self.hand_1 = self.env.GetRobots()[0]
        self.hand_2 = self.env.GetRobots()[1]
        self.flag = False
        self.hand_1_mat = np.array(self.hand_1.GetLinkTransformations())
        self.hand_2_mat = np.array(self.hand_2.GetLinkTransformations())
        self.part_mat = np.array(self.obj.GetTransform())
        self.hand_2.SetVisible(0)
        self.obj.SetVisible(1)

    def Switch_transform(self,transform_1,transform_2):
        print type(transform_2)
        print type(transform_1[1])
        return_vec =[]
        for i in range(14):
            print transform_1[i]
            print ""
            temp_matrix = np.dot(np.linalg.inv(transform_1[i]),transform_2)
            print temp_matrix
            return_vec.append(temp_matrix)
            print return_vec
        return return_vec

    def hand_transformer(self, hand_mat):
        temp_matrix = hand_mat.data
        new_temp = temp_matrix[0:-1]
        new_temp = np.array(new_temp)
        self.hand_1_mat = new_temp.reshape(14,4,4)
        if temp_matrix[-1]==1:
            self.flag=True
        else:
            self.flag=False

    def part_transformer(self, part_mat):
        temp_matrix = part_mat.data
        temp_matrix = np.array(temp_matrix)
        self.part_mat = temp_matrix.reshape((4,4))

    def generate_environment(self):
        try:
            while not rospy.is_shutdown():
                hand_wrt_obj = self.Switch_transform(self.hand_1_mat,self.part_mat)
                hand_wrt_obj = np.array(hand_wrt_obj)
                self.hand_1.SetLinkTransformations(hand_wrt_obj)
                if self.flag==True:
                    self.hand_2.SetLinkTransformations(hand_wrt_obj)
                    self.hand_2_mat = hand_wrt_obj
                else:
                    self.hand_2.SetLinkTransformations(self.hand_2_mat)
        except KeyboardInterrupt, e:
            print "exiting due to ",e
        

def main():
    ctrl = object_visualizer()
    rospy.init_node('object_visualizer',anonymous = True)
    ctrl.sub = rospy.Subscriber("/hand_transformation", Float32MultiArray, ctrl.hand_transformer)
    ctrl.sub_trans = rospy.Subscriber("/obj_transformation",Float32MultiArray, ctrl.part_transformer)
    ctrl.generate_environment()

if __name__=="__main__":
    main()
