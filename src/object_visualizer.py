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
import mdp
import math
from openravepy.examples import tutorial_grasptransform


from obj_dict import *

class object_visualizer(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path('barrett_hand_control')
	self.stl_path = self.path + "/src/stl_files"
        self.env = Environment()
	self.obj_num = 15
        self.env.Load(self.stl_path + '/Chunk_of_foam.STL',{'scalegeometry':'0.001 0.001 0.001'})
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
	self.gt = tutorial_grasptransform.GraspTransform(self.env, self.hand_1)

    def set_obj(self, obj_num):
    	global grasp_obj_dict
	self.obj_num = obj_num
    	self.env.Remove(self.obj)
	self.env.Load(self.stl_path + "/" + grasp_obj_dict[obj_num][1], {'scalegeometry':'0.001 0.001 0.001'})
	self.obj = self.env.GetKinBody(grasp_obj_dict[obj_num][1].split('.')[0])
	self.obj_y_rotate = grasp_obj_dict[obj_num][2]
	T_cent = self.get_stl_centroid_transform()
	self.apply_link_transform(T_cent, self.obj)
	
	rospy.loginfo("Loaded " + grasp_obj_dict[obj_num][0])

    def reorient_hand(self, T_palm, T_obj):
	self.obj.SetVisible(1)
	hand_to_obj = np.dot(np.linalg.inv(T_obj), T_palm)

	self.hand_1.SetTransform(hand_to_obj)

	self.standard_axes = self.gt.drawTransform(np.eye(4))
	self.recenter_from_stl()
	self.standardize_axes()
	palm_pt = self.get_palm_point()
	self.palm_plot = self.env.plot3(palm_pt, 10)
	print "Final palm point: ", palm_pt[0], "\t", palm_pt[1], "\t", palm_pt[2]
	raw_input("Finished reorientation. How are we doing?")

   
    # Recenters the robot hand relative to
    #	the centroid of the object
    def recenter_from_stl(self):
	T_cent = self.get_stl_centroid_transform()
	self.apply_link_transform(T_cent, self.hand_1)

    def get_stl_centroid_transform(self):
    	global obj_centroid_dict
	centroid = obj_centroid_dict[self.obj_num]
	T_cent = np.eye(4)
	for idx, x in enumerate(centroid):
		T_cent[idx][3] = x / 1000.0
	T_cent = np.linalg.inv(T_cent)
	
	return T_cent

    # If the object can freely rotate about the y axis, all grasps should
    #	be reoriented into the same coordinate frame. This function will
    #	standardize the frame of reference.
    def standardize_axes(self):
    	if not self.obj_y_rotate:
		rospy.loginfo("No axis standardization necessary.")
		return
	
	# Ensure that the grasp is pointing toward the principle component
	#p = mdp.nodes.PCANode(output_dim=3)
	#p.train(self.get_obj_points())
	#y = mdp.pca(self.get_obj_points())
	#print "pca results: ", dir(p)
	#print "pca y: ", p.get_recmatrix()
	#self.plot_h = self.env.plot3(p.get_recmatrix(),6)

	palm_pt = self.get_palm_point()
	palm_approach = [0,0,1] # Here is the palm's direction in its own corrdinate frame
	palm_approach = poseTransformPoints(poseFromMatrix(self.hand_1.GetTransform()), [palm_approach])[0]
	print "palm_approach: ", palm_approach
	try:
		angle = get_angle([0,1,0], palm_approach)
	except:
		rospy.loginfo("Angles too close to consider. Probably dont need to realign.")
		return
	#print "angle: ", angle
	if angle > (math.pi/3) and angle < (2*math.pi/3):
		pass
	else:
		rospy.loginfo("Approach is not along the rotational axis. Skipping.")
		return

	# Find the xz-planar angular difference between the x axis and the palm point
	angle = get_angle([palm_pt[0], palm_pt[2]], [1,0])
	if palm_pt[2] < 0:
		angle = -angle

	# Get the rotation matrix about the y axis
	print "Angle:", angle
	standardize_mat = matrixFromAxisAngle([0,1,0], angle)

	# Reorient things...
	self.apply_scene_transform(standardize_mat)
	rospy.loginfo("Axes standardized.")

    def apply_scene_transform(self, T):
    	self.apply_link_transform(T, self.hand_1)
	self.apply_link_transform(T, self.obj)

    def apply_link_transform(self, T, link):
    	T_l = link.GetTransform()
	T_l_new = np.dot(T, T_l)
	link.SetTransform(T_l_new)

    def get_palm_point(self):
	palm_pose = poseFromMatrix(self.hand_1.GetTransform())
	palm_pt = numpy.array(palm_pose[4:])
	palm_pt += self.get_palm_offset()
	#print "Point on palm", palm_pt

	return palm_pt

    # Finds a vector from the base of the wrist to the middle of he palm by following
    #	the apporach vector for the depth of the palm (7.5cm)
    def get_palm_offset(self):
    	palm_approach = [0,0,1]
	palm_approach = poseTransformPoints(poseFromMatrix(self.hand_1.GetTransform()), [palm_approach])[0]
	palm_approach = np.array(palm_approach)
	palm_approach = palm_approach / np.linalg.norm(palm_approach)
	palm_approach *= 0.075
	return palm_approach

    def get_obj_points(self):
    	obj_pts = self.obj.GetLinks()[0].GetCollisionData().vertices
	obj_pose = poseFromMatrix(self.obj.GetTransform())

	return  poseTransformPoints(obj_pose, obj_pts)

# Returns the angular difference between the vectors in radians
def get_angle(v1, v2):
	n1 = float(np.linalg.norm(v1))
	n2 = float(np.linalg.norm(v2))
	if n1 < 0.001 or n2 < 0.001:
		print "The vectors are too small to compare."
		raise ValueError

	return np.arccos(np.dot(v1, v2) / (n1 * n2))

def get_transforms(file_path):
	f = open(file_path, "r")
	lines = f.readlines()
	f.close()
	mats = [0,0]
	mats[0] = "".join(lines[1:5]).replace("[", "").replace("]","").split("\n")
	mats[1] = "".join(lines[7:]).replace("[","").replace("]", "").split("\n")
	
	#print "obj premat", mats[0] 
	#print "hand premat", mats[1]
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
    while not rospy.is_shutdown():
    	obj_num = int(raw_input("Obj num: "))
	sub_num = int(raw_input("Sub num: "))

	#transform_path = "/media/eva/FA648F24648EE2AD" + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
	#transform_path = os.path.expanduser("~") + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
	transform_path = os.path.expanduser("~") + "/grasp_transforms"
	files = os.listdir(transform_path)
	files = sorted(files)
	ctrl.set_obj(obj_num)
	for f in files:
		f = transform_path + "/" + f
		if ("obj" + str(obj_num)) in f and "object_transform" in f:
			rospy.loginfo("Showing " + f)
			T_hand, T_obj = get_transforms(f)
			ctrl.reorient_hand(T_hand, T_obj)

if __name__=="__main__":
    main()
