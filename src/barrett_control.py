#!/usr/bin/env python

from openravepy import *
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from grasp_manager.msg import GraspSnapshot
import sys
import rospkg
import time
import csv
import os
import threading
from Tkinter import *


class control(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path('barrett_hand_control')
        self.flag = 0
        self.env = Environment()
        self.env.Load(self.path+'/src/barrett_wam.dae')
        self.robot = self.env.GetRobots()[0]
        self.robot.SetTransform([[1,0,0,0],[0,1,0,0],[0,0,1,-1.16],[0,0,0,1]])
        self.env.SetViewer('qtcoin')
        self.pub = rospy.Publisher('hand_transformation', Float32MultiArray,queue_size = 1)
        self.pub_obj = rospy.Publisher('obj_transformation',Float32MultiArray,queue_size = 1)
        self.point_cloud = np.array([])
        self.obj_number = 6
        self.sub_number = 7
        self.grasp_set = 7
        self.extreme = 0
        self.master = Tk()
#        self.capturing_directory = ()
        self.is_data_loaded = 0
        self.color_vector = np.array([])
        self.plot_point_cloud = None
        self.transform_points = np.array([])
        self.is_optimal = False
        self.optimal_num =1 
        self.new_path = '/home/saurabh/csvfiles'

    def User_input(self,snap_shot):
        self.obj_number = snap_shot.obj_num
        self.sub_number = snap_shot.sub_num
        self.grasp_set = snap_shot.grasp_num
        self.extreme = snap_shot.extreme_num
        self.is_optimal = snap_shot.is_optimal
        if self.is_optimal:
            self.optimal_num = snap_shot.optimal_num
            filename = (self.new_path+'/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_pointcloud_csvfiles/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_grasp'+str(self.grasp_set)+'_optimal'+str(self.optimal_num)+'_pointcloud.csv')
        else:
            filename = (self.new_path+'/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_pointcloud_csvfiles/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_grasp'+str(self.grasp_set)+'_extreme'+str(self.extreme)+'_pointcloud.csv')

        self.point_cloud = []
        del self.plot_point_cloud
        try:
            with open(filename,'rb') as csvfile:
                pointcloud = csv.reader(csvfile, delimiter=',', quotechar = '|')                                            
                for row in pointcloud:                                                                                      
                    row_float = []                                                                                          
                    for value in row:                                                                                       
                        row_float.append(float(value))                                                                      
                    self.point_cloud.append(row_float)                                                                      
            self.point_cloud = np.array(self.point_cloud)
            self.color_vector = self.point_cloud[:,3:6]
            self.color_vector = np.divide(self.color_vector,255.0)
            point_file = open(self.path+'/src/MasterMatrix.txt','rb')
            string = point_file.read()
            string = string.replace(',','\n')
            vec = string.split()
            empty_vec = []
            for value in vec:
                empty_vec.append(float(value))

            transformation_matrix = np.array(empty_vec)
            transformation_matrix = transformation_matrix.reshape(4,4)
            self.transform_points = poseTransformPoints(transformation_matrix,self.point_cloud[:,0:3])
            #self.plot_point_cloud = self.env.plot3(self.transform_points,6,self.color_vector)
            self.plot_point_cloud = 0
        except IOError, e:
            print e

    def generate_environment(self):
        try:
            self.obj =self.env.ReadKinBodyXMLFile(self.path+'/src/stl_files/CrackerBox.STL',{'scalegeometry':'0.001 0.001 0.001'})
            self.env.Add(self.obj)
            self.Table = self.env.ReadKinBodyXMLFile('data/table.kinbody.xml')
            self.env.Add(self.Table)
            self.Table.SetTransform([[0.0007963267271406949, -0.9999997019767761, 0.0, -0.8919000029563904], [0.9999997019767761, 0.0007963267271406949, 0.0, 0.3614000082015991], [-0.0, 0.0, 1.0, 1.0058000087738037-1.16], [0.0, 0.0, 0.0, 0.0]])
            self.Trans_matrix = np.array([[0.99713,-0.026123,0.071081,-0.81319],[0.063927,-0.21283,-0.975,-0.1691],[0.040598,0.97674,-0.21055,-0.10489],[0,0,0,1]])
            print self.Trans_matrix
            transposed_rotation_matrix = np.transpose(self.Trans_matrix[0:3,0:3])
            col_vector = self.Trans_matrix[:,-1].reshape(4,1)
            self.Trans_matrix = np.append(transposed_rotation_matrix, [self.Trans_matrix[-1,0:3]],axis = 0)
            self.Trans_matrix = np.append(self.Trans_matrix, col_vector, axis=1)
            self.obj.SetTransform(self.Trans_matrix)
            print self.Trans_matrix
            self.env.Remove(self.Table)
            while not rospy.is_shutdown():
                hand_transformation_vec = []
                vec = self.robot.GetLinkTransformations()[9:23]
                for i in range(len(vec)):
                    temp_vec = []
                    temp_vec = vec[i].reshape(-1)
                    temp_vec = temp_vec.tolist()
                    hand_transformation_vec += temp_vec
                hand_transformation_vec.append(0)
                if self.flag==1:
                    hand_transformation_vec[-1] = 1
                else:
                    hand_transformation_vec[-1] = 0
                self.pub.publish(data=hand_transformation_vec)
                vec_obj = self.obj.GetTransform()
                vec_obj = np.array(vec_obj)
                obj_transformation_vec = vec_obj.reshape(-1)
                self.pub_obj.publish(data = obj_transformation_vec)
                self.master.update()
        except KeyboardInterrupt,e:
            print e
        finally:
            print 'exiting'
            self.env.Destroy()

    def updater(self,snapshot):
        T_hand = np.array(snapshot.hand_joints.position)
        T_wam = np.array(snapshot.wam_joints.position)
        T_robot = T_wam[0:7]
        T_robot = np.append(T_robot,[0,0])
        T_robot = np.append(T_robot,[T_hand[3],T_hand[0],T_hand[4],T_hand[3],T_hand[1],T_hand[5],T_hand[2],T_hand[6]])
        self.robot.SetDOFValues(T_robot)
    
    def generate_file(self):
        self.master.button = Button(self.master, text = "Generate File for current object transformation", height =10, width=20, command = self.makefile)
        self.master.button.pack()

    def makefile(self):
        if self.is_optimal:
            filename = open(self.new_path+'/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_pointcloud_csvfiles/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_grasp'+str(self.grasp_set)+'_optimal'+str(self.optimal_num)+'_object_transform.txt','wb')
        else:
            filename = open(self.new_path+'/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_pointcloud_csvfiles/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_grasp'+str(self.grasp_set)+'_extreme'+str(self.extreme)+'_object_transform.txt','wb')
        obj_transform = self.obj.GetTransform()
        link_transformations = self.robot.GetLinkTransformations()
        hand_transform = link_transformations[9]
        obj_transform = "object_transform \n"+str(obj_transform)
        new_string  = obj_transform+"\n\nHand_Transformation\n"+str(hand_transform)
        print new_string
        print "written in ",filename
        print self.is_optimal
        filename.write(new_string)
        filename.close()

    def updater_part(self,transform_values):
        transform = [[transform_values.data[0],transform_values.data[1],transform_values.data[2],transform_values.data[3]],[transform_values.data[4],transform_values.data[5],transform_values.data[6],transform_values.data[7]],[transform_values.data[8],transform_values.data[9],transform_values.data[10],transform_values.data[11]],[transform_values.data[12],transform_values.data[13],transform_values.data[14],transform_values.data[15]]]

        self.obj.SetTransform(transform)


def main():
    ctrl = control()
    rospy.init_node('control',anonymous = True)
    ctrl.sub = rospy.Subscriber("grasp_extremes", GraspSnapshot, ctrl.updater)
    ctrl.sub_trans = rospy.Subscriber("slider_for_transformation",Float32MultiArray, ctrl.updater_part)
    ctrl.sub_grasp_extremes = rospy.Subscriber("grasp_extremes",GraspSnapshot,ctrl.User_input)
    ctrl.generate_file()
    ctrl.generate_environment()

if __name__=="__main__":
    main()
