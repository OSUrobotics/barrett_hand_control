#! /usr/bin/python
import rosbag
import rospy

import os
import csv
import copy

from object_visualizer import *
from stl_generator import get_robot_points

base_path = os.path.expanduser("~")
contact_dir = base_path + "/grasp_contacts"
joint_angle_master_path = base_path + "/grasp_results/grasp_joints_joint_angle_master.csv"
transform_path = base_path + "/grasp_transforms"

# This number is the index of the first palm point in the full robot stl
#	it will be used to offset the given vertices into just the hand mesh
stl_vertex_offset = 243924
interesting_vertex_list = [('PalmCenter',248168), ('PalmCorner1',248242), ('PalmCorner2',248304), ('PalmCorner3',247847), ('PalmCorner4', 248011), ('ThumbTip', 417162), ('ThumbJoint1',400798), ('ThumbJoint2',403995), ('Finger1Tip',386510), ('Finger1Joint1',379810), ('Finger1Joint2',374084), ('Finger2Tip',347470), ('Finger2Joint1',328584), ('Finger2Joint2',336576)]

def aggregate_data(similiar_in, similar_out_csv, out_dict):
	global contact_dir, joint_angle_master_path, robot_reorientor, transform_path, stl_vertex_offset, interesting_vertex_list
	# For each specified grasp, get the object, get the subject, grasp num, etc...
	for grasp_spec_dict in similar_in_csv:
		obj_num = out_dict['object'] = int(grasp_spec_dict['object'])
		sub_num = out_dict['subject'] = int(grasp_spec_dict['subject'])
		grasp_num = out_dict['grasp'] = int(grasp_spec_dict['grasp'])
		o_or_e = out_dict['extreme or optimal'] = grasp_spec_dict['extreme or optimal']
		idx = out_dict['number'] = int(grasp_spec_dict['number'])

		base_name = std_file_prefix(obj_num, sub_num, grasp_num, o_or_e, idx)
		rospy.loginfo("Processing " + base_name)

		# Get contact points
		contact_path = contact_dir + "/" + "obj" + str(obj_num) + "_sub" + str(sub_num) + "_all_grasps_contact_points.csv"
		contact_dict_list = load_csv_dict_list(contact_path)
		found_pts = False
		for d in contact_dict_list:
			if int(d['Object Number ']) == obj_num and int(d['Subject Number']) == sub_num and int(d['Grasp number']) == grasp_num and d['extreme or optimal'] == o_or_e and int(d['number']) == idx:
				# Then we got our grasp.
				found_pts = True
				mesh_dictionaries(out_dict, d)

		if not found_pts:
			rospy.logerr("Did not find contact points for " + std_file_prefix(obj_num, sub_num, grasp_num, o_or_e, idx))

		# Get joint angles
		angle_dict_list = load_csv_dict_list(joint_angle_master_path)
		found_jnts = False
		for d in angle_dict_list:
			if grasp_spec_dict['object'] == d['Object'] and grasp_spec_dict['subject'] == d['Subject'] and grasp_spec_dict['grasp'] == d['Grasp'] and grasp_spec_dict['extreme or optimal'] == d['Optimality'] and grasp_spec_dict['number'] == d['Index'] and d['Good or Bad'] == "good":
				# We also got our grasp
				found_jnts = True
				mesh_dictionaries(out_dict, d)

		if not found_jnts:
			rospy.logerr("Did not find joint angles for " + std_file_prefix(obj_num, sub_num, grasp_num, o_or_e, idx))

		# Normalize those joint angles
		for k in out_dict:
			if k == "inner_f1_norm":
				out_dict[k] = float(out_dict['inner_f1']) / 2.443460952792061
			elif k == "inner_f2_norm":
				out_dict[k] = float(out_dict['inner_f2']) / 2.443460952792061
			elif k == "inner_f3_norm":
				out_dict[k] = float(out_dict['inner_f3']) / 2.443460952792061
			elif k == "outer_f1_norm":
				out_dict[k] = float(out_dict['outer_f1']) / 1.1
			elif k == "outer_f2_norm":
				out_dict[k] = float(out_dict['outer_f2']) / 1.1
			elif k == "outer_f3_norm":
				out_dict[k] = float(out_dict['outer_f3']) / 1.1
			elif k == "spread_norm":
				out_dict[k] = float(out_dict['spread']) / 3.14159

		# Get those pesky XYZ positions: Needs to be done last because there are multiple xyz positions per grasp
		robot_reorientor.set_obj(obj_num)
		obj_transform_path = transform_path + "/" + base_name + "_object_transform.txt"
		T_hand, T_obj = get_transforms(obj_transform_path)
		icp_T_hand = get_icp_transform(base_name)
		if icp_T_hand is not None:
			T_obj = icp_T_hand
		robot_reorientor.set_hand_joints(out_dict)
		robot_reorientor.reorient_hand(T_hand, T_obj)
		pts = get_robot_points(robot_reorientor.hand_1)[0]
		plot_pts = []
		for i in interesting_vertex_list:
			n = i[0]
			idx = i[1] - stl_vertex_offset
			if idx < 0:
				rospy.logerr("Trying to access non-existant vertex on hand.")
				continue
			p = pts[idx]
			out_dict['xyz name'] = n
			out_dict['x'] = p[0]
			out_dict['y'] = p[1]
			out_dict['z'] = p[2]
			similar_out_csv.writerow(out_dict)
			plot_pts.append(robot_reorientor.env.plot3(p, 10))
			#raw_input("Added point " + n)

		#raw_input("How are those points?")


def get_icp_transform(base_name):
	global transform_path
	icp_path = transform_path + "/icp_results/" + base_name + "_obj_align_matrix.txt"
	if os.path.exists(icp_path):
		rospy.loginfo("Attempting icp load for " + base_name)
		with open(icp_path,"r") as icp_f:
			mat_str = icp_f.read()
			mat = mat_str.split()
			mat = np.array(map(float, mat))
			mat = mat.reshape(4,4) # Cool =)
			return mat
	else:
		return None

def std_file_prefix(obj, sub, grasp, oe, idx):
	return ("obj" + str(obj) + "_sub" + str(sub) + "_grasp" + str(grasp) + "_" + oe + str(idx))

# Just loads a csv file into memory as a list of
#	dictionaries.
def load_csv_dict_list(csv_path):
	out_list = []
	with open(csv_path, "r") as csvfile:
		in_file = csv.DictReader(csvfile, delimiter=",")
		for d in in_file:
			out_list.append(d)

	return out_list

# Attempts to put the values of all matching keys of src_d into dest_d
#	with the same keys
def mesh_dictionaries(dest_d, src_d):
	for k in dest_d:
		if k in src_d:
			dest_d[k] = src_d[k]
	return dest_d

if __name__ =="__main__":
	rospy.init_node("final_data_aggregator")
	rospy.loginfo("Data aggregator online.")
	master_field_dict = {'object':'', 'subject':'', 'grasp':'', 'extreme or optimal':'', 'number':'', 'xyz name':'', 'x':'', 'y':'', 'z':'', 'inner_f1':'', 'inner_f2':'', 'inner_f3':'', 'outer_f1':'', 'outer_f2':'', 'outer_f3':'', 'spread':'', 'inner_f1_norm':'', 'inner_f2_norm':'', 'inner_f3_norm':'', 'outer_f1_norm':'', 'outer_f2_norm':'', 'outer_f3_norm':'', 'spread_norm':'', 'F1 Tip':'', 'F2 Tip':'', 'F3 Tip':'', 'F1 Pad':'', 'F2 Pad':'', 'F3 Pad':'', 'F1 Joint':'', 'F2 Joint':'', 'F3 Joint':'', 'Contact on palm': ''}

	field_list = ['object', 'subject', 'grasp', 'extreme or optimal', 'number', 'xyz name', 'x', 'y', 'z', 'inner_f1', 'inner_f2', 'inner_f3', 'outer_f1', 'outer_f2', 'outer_f3', 'spread', 'inner_f1_norm', 'inner_f2_norm', 'inner_f3_norm', 'outer_f1_norm', 'outer_f2_norm', 'outer_f3_norm', 'spread_norm', 'F1 Tip', 'F2 Tip', 'F3 Tip', 'F1 Pad', 'F2 Pad', 'F3 Pad', 'F1 Joint', 'F2 Joint', 'F3 Joint', 'Contact on palm' ]
	robot_reorientor = object_visualizer()

	# Get the grasps to aggregate
	similar_dir = os.path.expanduser("~") + "/grasp_similarities"
	print "Similar dir: ", similar_dir
	for f in sorted(os.listdir(similar_dir)):
		similar_path = similar_dir + "/" + f
		if "output" in f or os.path.isdir(similar_path):
			continue

		# This file contains human defined similar grasps
		similar_result_path = similar_path.split(".")[0] + "_output.csv"
		similar_desc = open(similar_path, "r")
		similar_result = open(similar_result_path, "w")
		similar_in_csv = csv.DictReader(similar_desc, delimiter=",")
		similar_out_csv = csv.DictWriter(similar_result, field_list, delimiter=",")
		similar_out_csv.writeheader()
		
		aggregate_data(similar_in_csv, similar_out_csv, copy.deepcopy(master_field_dict))
		
		similar_desc.close()
		similar_result.close()

	rospy.loginfo("Data aggregation complete.")
