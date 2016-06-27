#!/usr/bin/python

from openravepy import *
import rospy
import numpy as np
import csv
from stlwriter import *

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
        faces_points = []
        for vec in self.indices:
            faces_points.append([self.vertices[vec[0]],self.vertices[vec[1]],self.vertices[vec[2]]])
        
        with open('object.stl','wb') as fp:
            writer= Binary_STL_Writer(fp)
            writer.add_faces(faces_points)
            writer.close()
if __name__=="__main__":
    rospy.init_node('stl_generator',anonymous=True)
    generate_stl = stl_generator()
    generate_stl.generate_env()



