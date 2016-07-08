#!/usr/bin/env python
from object_visualizer import *

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
		if ("obj" + str(obj_num)) in f and ("sub" + str(sub_num)) in f and "object_transform" in f:
			rospy.loginfo("Showing " + f)
			T_hand, T_obj = get_transforms(f)
			ctrl.reorient_hand(T_hand, T_obj)

if __name__=="__main__":
    main()