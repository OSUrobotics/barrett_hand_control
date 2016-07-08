from openravepy import *
import rospy
import numpy as np
import csv
from stlwriter import *
import os
#from stl import mesh

import rospkg

class stl_generator():
    def __init__(self, get_robot_points):
        self.env = Environment()
        self.get_robot_points = get_robot_points
        #self.viewer = self.env.SetViewer('qtcoin')
        bhc_path = rospkg.RosPack().get_path('barrett_hand_control')
        robot_path = bhc_path + '/src/barrett_wam.dae'
        self.env.Load(robot_path)
        self.robot = self.env.GetRobots()[0]
        self.robot.SetTransform([[1,0,0,0],[0,1,0,0],[0,0,1,-1.16],[0,0,0,1]])
        self.vertices = np.array([])
        self.indices = np.array([])
        self.links = self.robot.GetLinks()

    def set_joints(self, hand_joints, wam_joints):
        T_hand = np.array(hand_joints)
        T_wam = np.array(wam_joints)
        T_robot = T_wam[0:7]
        T_robot = np.append(T_robot,[0,0])
        T_robot = np.append(T_robot,[T_hand[3],T_hand[0],T_hand[4],T_hand[3],T_hand[1],T_hand[5],T_hand[2],T_hand[6]])
        self.robot.SetDOFValues(T_robot)

    def generate_stl(self, stl_out_path):
        if os.path.exists(stl_out_path):
            rospy.loginfo("STL already exists: " + stl_out_path)
            return
        all_vertices, all_faces = self.get_robot_points(self.robot)

        self.vertices = numpy.array(all_vertices)
        self.indices = numpy.array(all_faces)

        self.write_stl(stl_out_path)

    # Now try reading the same stl
    #robo_mesh = mesh.Mesh.from_file(stl_out_path)
    #print "Num stl vertices: ", len(robo_mesh.points)
    #print "Num vertices in trimesh: ", len(self.vertices)
    #print "first couple of trimesh points: ", self.vertices[0:6]
    #print "first couple of stl points: ", robo_mesh.points[0:6]
    #self.a = self.env.plot3(self.vertices[336576], 6)
    #self.b = self.env.plot3(self.vertices[403995], 6)
    #self.c = self.env.plot3(self.vertices[400798], 20)
    #raw_input("How do those numbers compare?")

    def write_stl(self, stl_out_path):
        faces_points = []
        #print "self.indices: ", self.indices
        #print "self.vertices: ", self.vertices
        for vec in self.indices:
            faces_points.append([self.vertices[vec[0]],self.vertices[vec[1]],self.vertices[vec[2]]])

        with open(stl_out_path,'wb') as fp:
            writer= Binary_STL_Writer(fp)
            writer.add_faces(faces_points)
            writer.close()
