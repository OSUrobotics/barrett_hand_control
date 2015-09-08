#!/usr/bin/env python
import threading
from openravepy import *
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import sys
import rospkg
import time
import os
import string

class object_visualizer(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path('barrett_hand_control')
        self.env = Environment()
        self.env.Load(self.path+'/src/stl_files/Chunk_of_foam.STL',{'scalegeometry':'0.001 0.001 0.001'})
        self.env.SetViewer('qtcoin')
        self.obj = self.env.GetBodies()[0]
        error_1 = self.env.Load(self.path+'/src/bhand.dae')
        error_2 = self.env.Load(self.path+'/src/new_bhand.dae')
        self.hand_1 = self.env.GetRobots()[0]
        self.hand_2 = self.env.GetRobots()[1]
        self.flag = False
        self.hand_1_mats = self.hand_1.GetLinkTransformations()
        self.hand_2_mats = self.hand_2.GetLinkTransformations()
        self.part_mat = np.array(self.obj.GetTransform())
        self.hand_2.SetVisible(0)
        self.obj.SetVisible(0)
	self.obj.SetTransform(np.eye(4))

    def Switch_transform(self, link_mats, obj_mat):
        print type(obj_mat)
        print type(link_mats[1])
        return_vec =[]
        for i in range(len(link_mats)):
            print "Link ", i, " matrix: ", link_mats[i], "\n"
            new_link_matrix = np.dot(mat_to_apply, link_mats[i])
            print "Transformed: ", new_link_matrix, "\n"
            return_vec.append(new_link_matrix)
        
	return return_vec

    def hand_transformer(self, hand_mat):
        temp_matrix = hand_mat.data
        new_temp = temp_matrix[0:-1]
        new_temp = np.array(new_temp)
        self.hand_1_mats = new_temp.reshape(14,4,4)
        if temp_matrix[-1]==1:
            self.flag=True
        else:
            self.flag=False

    def part_transformer(self, part_mat):
        rospy.loginfo("Got part transformation.")
	temp_matrix = part_mat.data
        temp_matrix = np.array(temp_matrix)
        self.part_mat = temp_matrix.reshape((4,4))

    def reorient_hand(self, T_palm, T_obj):
	mat_to_apply = np.dot(np.linalg.inv(T_obj), T_palm)

	#hand_wrt_obj = self.transform_hand_links(self.hand_1_mats,mat_to_apply)
	#self.hand_1.SetLinkTransformations(hand_wrt_obj)
	self.hand_1.SetTransform(mat_to_apply)
	#if self.flag==True:
	#    self.hand_2.SetLinkTransformations(hand_wrt_obj)
	#    self.hand_2_mats = hand_wrt_obj
	#else:
	#    self.hand_2.SetLinkTransformations(self.hand_2_mats)
	self.obj.SetVisible(1)
	raw_input("How does that look?")

def get_transforms(file_path):
	f = open(file_path, "r")
	lines = f.readlines()
	f.close()
	print lines
	print len(lines)
	mats = [0,0]
	print 
	mats[0] = "".join(lines[1:5]).replace("[", "").replace("]","").split("\n")
	mats[1] = "".join(lines[7:]).replace("[","").replace("]", "").split("\n")
	
	print "obj premat", mats[0] 
	print "hand premat", mats[1]
	obj_mat = []
	hand_mat = []
	for line in mats[0]:
		try:
			obj_mat.append(get_list_from_str(line))
		except:
			continue

	for line in mats[1]:
		try:
			hand_mat.append(get_list_from_str(line))
		except:
			continue
	
	if len(hand_mat) != 4 or len(hand_mat[0]) != 4:
		rospy.logerr("Didnt read in hand matrix properly!: " + str(hand_mat))
		sys.exit(1)
	if len(obj_mat) != 4 or len(obj_mat[0]) != 4:
		rospy.logerr("Didnt read in obj matrix properly: " + str(obj_mat))
		sys.exit(1)

	return numpy.array(hand_mat), numpy.array(obj_mat)

def get_list_from_str(in_str):
	if in_str == '':
		raise ValueError
	l = []
	split_str = in_str.split(" ")
	for n in split_str:
		if n is not "":
			l.append(float(n))
	return l

def main():
    ctrl = object_visualizer()
    rospy.init_node('object_visualizer',anonymous = True)
    #ctrl.sub = rospy.Subscriber("/hand_transformation", Float32MultiArray, ctrl.hand_transformer)
    #ctrl.sub_trans = rospy.Subscriber("/obj_transformation",Float32MultiArray, ctrl.part_transformer)
    while not rospy.is_shutdown():
    	obj_num = int(raw_input("Obj num: "))
	sub_num = int(raw_input("Sub num: "))

	#transform_path = "/media/eva/FA648F24648EE2AD" + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
	transform_path = os.path.expanduser("~") + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
	files = os.listdir(transform_path)
	for f in files:
		f = transform_path + "/" + f
		if "object_transform" in f:
			rospy.loginfo("Showing " + f)
			T_hand, T_obj = get_transforms(f)
			ctrl.reorient_hand(T_hand, T_obj)

if __name__=="__main__":
    main()
