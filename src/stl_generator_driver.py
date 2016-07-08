#!/usr/bin/python
from openravepy import *
import rospy
import numpy as np
import csv
from stlwriter import *
import os
from stl_generator import stl_generator

import rospkg

from shared_playback import *

stl_dest_dir = os.path.expanduser("~") + "/grasp_stls"

def get_robot_points(robot):
    links = robot.GetLinks()
    all_vertices = []
    all_faces = []
    ind = 0
    for link in links:
        vertices = link.GetCollisionData().vertices
        faces = link.GetCollisionData().indices
        if ind == 0:
            faces = np.add(faces,ind)
        else:
            faces = np.add(faces,ind+1)
            try:
                ind = faces[-1][-1]
            except:
                pass


        #print "link: ", link, "\nStarting index for this link: ", len(all_vertices)
        link_pose = poseFromMatrix(link.GetTransform())
        transform_vertices = poseTransformPoints(link_pose, vertices)
        all_vertices.extend(transform_vertices.tolist())
        all_faces.extend(faces.tolist())

    return all_vertices, all_faces


def write_stls(grasp_data_directory, stl_generator):
    global stl_dest_dir
    data_dirs = get_data_dirs(grasp_data_directory)
    for (data_dir_path, obj_num, sub_num) in data_dirs:
        # Open the snapshot bag
        snapshot_bag = None
        try:
            snapshot_bag = try_bag_open(data_dir_path + "/grasp_extreme_snapshots.bag")
        except IOError:
            rospy.logerr("Could not open snapshot bag for directory: " + data_dir_path)
            continue

        # Replay all messages
        for topic, msg, t in snapshot_bag.read_messages():
            if "good" in data_dir_path:
                stl_path = stl_dest_dir + "/" + "good"
            else:
                stl_path = stl_dest_dir + "/" + "bad"

            stl_path += "/" + "obj" + str(obj_num) + "_sub" + str(sub_num) + "_grasp" + str(msg.grasp_num)
            if msg.is_optimal:
                stl_path += "_optimal" + str(msg.optimal_num)
            else:
                stl_path += "_extreme" + str(msg.extreme_num)
            stl_path += "_robot.stl"

            rospy.loginfo("Saving stl " + stl_path)
            stl_generator.set_joints(msg.hand_joints.position, msg.wam_joints.position)
            stl_generator.generate_stl(stl_path)

        snapshot_bag.close()

if __name__=="__main__":
    rospy.init_node('stl_generator',anonymous=True)
    rospy.loginfo("Stl generator script online.")
    rospy.loginfo("Saving stls to " + stl_dest_dir + " and reading data from " + grasp_data_directory)

    generate_stl = stl_generator(get_robot_points)

    if not os.path.exists(stl_dest_dir):
        rospy.loginfo("Stl saving path does not exist, creating it.")
        os.makedirs(stl_dest_dir + "/good")
        os.makedirs(stl_dest_dir + "/bad")

    write_stls(grasp_data_directory, generate_stl)
    rospy.loginfo("STL conversions complete.")
