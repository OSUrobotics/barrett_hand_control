#! /usr/bin/env python
import rospy
from openravepy import *
import numpy as np
import csv
from stlwriter import *
import os
from stl_generator import stl_generator
from get_matrix import *

from grasp_manager.shared_playback import *
stl_dest_dir = os.path.expanduser('~') + '/matts_stls'
palm_offset_to_origin = 0.077 #m between base of hand and origin of world

def offset(pt):
    global palm_offset_to_origin
    pt[2] = pt[2] - palm_offset_to_origin
    return pt

def get_robot_points(robot):
    # Get the hand links
    links = [l for l in robot.GetLinks() if 'bhand' in l.GetName()]
    #robot.SetTransformWithDOFValues(matrixFromPose(invertPoses([poseFromMatrix(links[0].GetTransform())])[0]), robot.GetDOFValues())
    #base_transform = links[0].GetTransformPose()
    #base_transform[:4] = quatInverse(base_transform)
    #base_transform[4:] = base_transform[4:] * -1
    #print "base_transform: ", base_transform
    all_vertices = []
    all_faces = []
    ind = 0
    for link in links:
        vertices = link.GetCollisionData().vertices
        faces = link.GetCollisionData().indices
        print "faces: ", len(faces), " vertices: ", len(vertices)
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
        print "link_pose: ", link_pose
        #p = poseMult(base_transform, link_pose)
        #p[4:] = link_pose[4:] + base_transform[4:]
        #print "p: ", p
        transform_vertices = poseTransformPoints(link_pose, vertices)
        #transform_vertices = map(offset, transform_vertices)

        all_vertices.extend(transform_vertices)
        all_faces.extend(faces.tolist())
        #print "all_vertices len: ", len(all_vertices), " all_faces len: ", len(all_faces)

    #print "all_vertices: ", all_vertices
    return all_vertices, all_faces

def write_stl_file_list(stl_file_list):
    global stl_dest_dir
    fname = stl_dest_dir + "/stl_file_list.txt"
    with open(fname, 'w') as f:
        f.write("\n".join(stl_file_list))

def write_stls(grasp_data_directory, stl_generator):
    global stl_dest_dir
    stl_file_list = []
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
            stl_path += ".stl"

            rospy.loginfo("Saving stl " + stl_path)
            stl_generator.set_joints(msg.hand_joints.position)
            #print "hand joints: ", msg.hand_joints
            #stl_generator.robot.SetDOFValues(np.array([0,0] + list(msg.hand_joints.position)))
            stl_generator.generate_stl(stl_path)
            stl_file_list.append(stl_path)
            write_transform(stl_path)

        snapshot_bag.close()
        write_stl_file_list(stl_file_list)

def write_transform(transform_dest_path):
    transform_fpath = "/media/sonny/FA648F24648EE2AD/grasp_study_2015/obj_transforms/" + transform_dest_path.split("/")[-1].replace(".stl", "_object_transform.txt")
    transform_dest_path = transform_dest_path.replace(".stl", "_transform.txt")
    try:
        m = get_matrix(transform_fpath)
    except IOError:
        rospy.logerr("No transform found. FIX THIS.")
        return
    print "hand_mat: ", m['hand_matrix'], "obj_mat: ", m['obj_matrix']
    hto = np.linalg.inv(np.array(m['hand_matrix'])) * np.array(m['obj_matrix'])
    with open(transform_dest_path, 'w') as f:
        for i in range(4):
            f.write("%s, %s, %s, %s\n" % (str(hto[i][0]), str(hto[i][1]), str(hto[i][2]), str(hto[i][3])))

if __name__=="__main__":
    rospy.init_node('stl_generator_graspwiggle',anonymous=True)
    rospy.loginfo("Stl generator script online.")
    rospy.loginfo("Saving stls to " + stl_dest_dir + " and reading data from " + grasp_data_directory)

    generate_stl = stl_generator(get_robot_points)

    if not os.path.exists(stl_dest_dir):
        rospy.loginfo("Stl saving path does not exist, creating it.")
        os.makedirs(stl_dest_dir + "/good")
        os.makedirs(stl_dest_dir + "/bad")

    write_stls(grasp_data_directory, generate_stl)
    rospy.loginfo("STL conversions complete.")
