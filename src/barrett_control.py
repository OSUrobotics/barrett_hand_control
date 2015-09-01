#!/usr/bin/env python
import threading
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

class control(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path('barrett_hand_control')
        self.new_path = '/home/saurabh/csvfiles'
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
        self.is_data_loaded = 0
        self.color_vector = np.array([])
        self.plot_point_cloud = None
        self.transform_points = np.array([])

    def User_input(self,snap_shot):
        print "running"
        self.obj_number = snap_shot.obj_num
        self.sub_number = snap_shot.sub_num
        self.grasp_set = snap_shot.grasp_num
        self.extreme = snap_shot.extreme_num
        self.point_cloud = self.point_cloud.tolist()
        try:
            with open(self.new_path+'/obj'+str(self.obj_number)+'_sub'+str(self.sub_number)+'_grasp'+str(self.grasp_set)+'_extreme'+str(self.extreme)+'_pointcloud.csv','rb') as csvfile:
                pointcloud = csv.reader(csvfile, delimiter=',', quotechar = '|')                                            
                for row in pointcloud:                                                                                      
                    row_float = []                                                                                          
                    for value in row:                                                                                       
                        row_float.append(float(value))                                                                      
                    self.point_cloud.append(row_float)                                                                      
            self.point_cloud = np.array(self.point_cloud)
            self.color_vector = self.point_cloud[:,3:6]
            self.color_vector = np.divide(self.color_vector,255.0)
            self.transform_points = poseTransformPoints([[-0.94368,0.098388,0.18805,-1.0602],[-0.08493,0.61037,-0.74553,0.77164],[-0.1945,-0.74388,-0.58686,0.84733],[0,0,0,1]],self.point_cloud[:,0:3])
            self.is_data_loaded = 1
        except IOError, e:
            print e

    def generate_environment(self):
        try:
            self.hand = self.env.ReadRobotXMLFile(self.path+'/src/bhand.dae')
            self.wam = self.env.ReadRobotXMLFile(self.path+'/src/wam.dae')
#            self.robot.SetDOFValues([8.29153121e-01, -1.46121848e+00, 2.03694558e+00, 1.35058486e+00, -9.14774418e-01, 5.70182264e-01, 2.35470676e+00, 2.22044605e-16, 2.22044605e-16, 1.56904329e-02, 1.49526310e+00, 4.98535573e-01, 1.56904329e-02, 1.45774198e+00, 4.86576647e-01, 1.29563081e+00, 5.63570201e-01])
            self.robot.SetDOFValues([0.81366723775863636, -1.5860003709793091, 2.0457030773162836, 1.2624661684036254, -0.89291913509368837, 0.62145200967788694, 2.4271213531494147, 3.1086244689504381e-16, 3.1086244689504381e-16, 0.015690432861447334, 1.4682650566101076, 0.48958203792572014, 0.015690432861447424, 1.4379965782165527, 0.47997848391532927, 1.3054973840713502, 0.53829675912857122])
            self.env.Add(self.hand)
            self.env.Add(self.wam)
            self.obj = self.env.ReadKinBodyXMLFile(self.path+'/src/stl_files/CrackerBox.STL',{'scalegeometry':'0.001 0.001 0.001'})
            self.env.Add(self.obj)
            self.Table = self.env.ReadKinBodyXMLFile('data/table.kinbody.xml')
            self.env.Add(self.Table)
            self.Table.SetTransform([[0.0007963267271406949, -0.9999997019767761, 0.0, -0.8919000029563904], [0.9999997019767761, 0.0007963267271406949, 0.0, 0.3614000082015991], [-0.0, 0.0, 1.0, 1.0058000087738037-1.16], [0.0, 0.0, 0.0, 0.0]])
            self.Trans_matrix = np.array([[-1.0, 3.140000104904175, -3.140000104904175, -0.71670001745224],[ 0.0, 0.0, -0.8199999928474426, -0.11140000075101852], [0.0, 0.0, 0.0, 1.0211999416351318-1.16], [0.0, 0.0, 0.0, 0.0]])
            self.obj.SetTransform(self.Trans_matrix)
            self.T_robot = self.robot.GetLinkTransformations()[9:23]
            self.hand.SetLinkTransformations(self.T_robot)
            self.T_wam = self.robot.GetLinkTransformations()[0:9]
            self.T_wam.append(self.robot.GetLinkTransformations()[-1])
            self.wam.SetLinkTransformations(self.T_wam)
            
            

            # Remove the following to visualize complete setup

            self.env.Remove(self.hand)
            self.env.Remove(self.Table)
            self.env.Remove(self.wam)
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
                while self.is_data_loaded != 1:
                    time.sleep(5)
                self.plot_point_cloud = self.env.plot3(self.transform_points,2,self.color_vector)
                self.pub_obj.publish(data = obj_transformation_vec)
        except KeyboardInterrupt,e:
            print e
        finally:
            print 'exiting'
            self.env.Destroy()

    def updater(self,slider_values):
        self.flag = slider_values.data[17]
        if self.flag == 1:
            self.T_robot = self.robot.GetLinkTransformations()[9:23]
            self.hand.SetLinkTransformations(self.T_robot)
            self.T_wam = self.robot.GetLinkTransformations()[0:9]
            self.T_wam.append(self.robot.GetLinkTransformations()[-1])
            self.wam.SetLinkTransformations(self.T_wam)
        self.robot.SetDOFValues(slider_values.data[0:17])
    
    def updater_part(self,transform_values):
        transform = [[transform_values.data[0],transform_values.data[1],transform_values.data[2],transform_values.data[3]],[transform_values.data[4],transform_values.data[5],transform_values.data[6],transform_values.data[7]],[transform_values.data[8],transform_values.data[9],transform_values.data[10],transform_values.data[11]],[transform_values.data[12],transform_values.data[13],transform_values.data[14],transform_values.data[15]]]

        self.obj.SetTransform(transform)


def main():
    ctrl = control()
    rospy.init_node('control',anonymous = True)
    ctrl.sub = rospy.Subscriber("control_slider_values", Float32MultiArray, ctrl.updater)
    ctrl.sub_trans = rospy.Subscriber("slider_for_transformation",Float32MultiArray, ctrl.updater_part)
    ctrl.sub_grasp_extremes = rospy.Subscriber("grasp_extremes",GraspSnapshot,ctrl.User_input)
    ctrl.generate_environment()

if __name__=="__main__":
    main()
