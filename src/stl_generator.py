from openravepy import *
import rospy
import numpy as np
import csv
from stlwriter import *
<<<<<<< HEAD

class stl_generator(object):
    def __init__(self):
        self.env = Environment()
        self.env.Load("/home/saurabh/research_environments/adept.dae")
        self.env.SetViewer('qtcoin')
        self.robot = self.env.GetRobots()[0]
        self.part = self.env.GetBodies()[1]
        self.vertices = np.array([])
        self.indices = np.array([])
        self.links = self.robot.GetLinks() 
        self.part_link = self.part.GetLinks()[0]

    def generate_env(self):
        try:
            all_vertices = []
            all_faces = []
            part_vertices = self.part_link.GetCollisionData().vertices
            part_faces = self.part_link.GetCollisionData().indices
            ind = part_faces[-1][-1]
	    part_link_pose = poseFromMatrix(self.part_link.GetTransform())
	    transform_part_vertices = poseTransformPoints(part_link_pose, part_vertices)
            all_vertices.extend(transform_part_vertices.tolist())
            all_faces.extend(part_faces.tolist())

            for link in self.links:
                vertices = link.GetCollisionData().vertices
                faces = link.GetCollisionData().indices
                if ind==0:
                    faces = np.add(faces,ind)
                else:
                    faces = np.add(faces,ind+1)
                try:
                    ind = faces[-1][-1]
                except:
                    pass
                link_pose = poseFromMatrix(link.GetTransform())
                transform_vertices = poseTransformPoints(link_pose, vertices)
                all_vertices.extend(transform_vertices.tolist())
                all_faces.extend(faces.tolist())

            self.vertices = numpy.array(all_vertices)
            self.indices = numpy.array(all_faces)
            pt_handles = self.env.plot3(self.vertices,4)
            filename = open('object.csv','wb')
            writer = csv.writer(filename, delimiter=',')
            writer.writerow(['x','y','z'])
            for row in self.vertices:
                writer.writerow(row)
            filename.close()
            self.write_stl()
            while not rospy.is_shutdown():
                n=1
        except KeyboardInterrupt, e:
            print "exiting due to ", e
    
    def write_stl(self):
=======
import os
#from stl import mesh

import rospkg

class stl_generator():
    def __init__(self, get_robot_points):
        self.env = Environment()
        self.get_robot_points = get_robot_points
        self.viewer = self.env.SetViewer('qtcoin')
        bhc_path = rospkg.RosPack().get_path('barrett_hand_control')
        robot_path = bhc_path + '/src/bhand.dae'
        self.env.Load(robot_path)
        self.robot = self.env.GetRobots()[0]
        self.robot.SetTransform([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.vertices = np.array([])
        self.indices = np.array([])
        self.links = self.robot.GetLinks()
        print "DOF: ", self.robot.GetDOFValues()

    def set_joints(self, hand_joints):
        T_hand = np.array(hand_joints)
        T_robot = np.array([])
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
>>>>>>> 788490ddbd6bc53b7c01cd845227898ea7fb2ff2
        faces_points = []
        print "self.indices: ", len(self.indices)
        print "self.vertices: ", len(self.vertices)
        for vec in self.indices:
            faces_points.append([self.vertices[vec[0]],self.vertices[vec[1]],self.vertices[vec[2]]])
<<<<<<< HEAD
        
        with open('object.stl','wb') as fp:
            writer= Binary_STL_Writer(fp)
            writer.add_faces(faces_points)
            writer.close()
if __name__=="__main__":
    rospy.init_node('stl_generator',anonymous=True)
    generate_stl = stl_generator()
    generate_stl.generate_env()



=======

        with open(stl_out_path,'wb') as fp:
            writer= Binary_STL_Writer(fp)
            writer.add_faces(faces_points)
            writer.close()
>>>>>>> 788490ddbd6bc53b7c01cd845227898ea7fb2ff2
