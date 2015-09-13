#!/usr/bin/env python
from object_visualizer import *

transform_path = os.path.expanduser("~") + "/grasp_transforms"
ctrl = None

def view_alignment_cb(msg):
	global transform_path, ctrl
	obj_num = msg.data[0]
	sub_num = msg.data[1]
	grasp_num = msg.data[2]
	idx = msg.data[3]

	f_name = "obj" + str(obj_num) + "_sub" + str(sub_num) + "_grasp" + str(grasp_num) + "_extreme" + str(idx)
	f_path = transform_path + "/" + f_name
	T_hand, T_obj = get_transforms(f)
	ctrl.reorient_hand(T_hand, T_obj)


def main():
    global transform_path, ctrl
    ctrl = object_visualizer()
    rospy.init_node('object_visualizer',anonymous = True)
    alignment_viewer_sub = rospy.Subscriber("/openrave_grasp_view", Int32MultiArray, view_alignment_cb)
    while not rospy.is_shutdown():
    	obj_num = int(raw_input("Obj num: "))
	sub_num = int(raw_input("Sub num: "))

	#transform_path = "/media/eva/FA648F24648EE2AD" + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
	#transform_path = os.path.expanduser("~") + "/csvfiles/obj" + str(obj_num) + "_sub" + str(sub_num) + "_pointcloud_csvfiles"
	files = os.listdir(transform_path)
	files = sorted(files)
	ctrl.set_obj(obj_num)
	for f in files:
		f = transform_path + "/" + f
		if ("obj" + str(obj_num)) in f and ("sub" + str(sub_num)) in f and "object_transform" in f:
			rospy.loginfo("Showing " + f)
			T_hand, T_obj = get_transforms(f)
			ctrl.reorient_hand(T_hand, T_obj)

if __name__=="__main__":
    main()
