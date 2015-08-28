#!/usr/bin/python

from openravepy import *
import rospy
import numpy as np
import csv

class stl_generator(object):
    def __init__(self):
        self.env = Environment()
        self.env.Load("/home/saurabh/stl_reader/obj6_subj7_grasp6.dae")
        self.env.SetViewer('qtcoin')
        self.robot = self.env.GetRobots()[0]
        self.vertices = np.array([[0,0,0]])
        self.indices = np.array([])
        self.links = self.robot.GetLinks() 

    def generate_env(self):
        try:
            all_vertices = []
            for link in self.links:
                vertices = link.GetCollisionData().vertices
                link_pose = poseFromMatrix(link.GetTransform())
                transform_vertices = poseTransformPoints(link_pose, vertices)
                #print transform_vertices
                all_vertices.extend(transform_vertices.tolist())
            self.vertices = numpy.array(all_vertices)
            pt_handles = self.env.plot3(self.vertices,4)
            print self.vertices
            print "shape",self.vertices.shape
            filename = open('obj6_sub7_grasp6.csv','wb')
            writer = csv.writer(filename, delimiter=',')
            writer.writerow(['x','y','z'])
            for row in self.vertices:
                writer.writerow(row)

            filename.close()
            while not rospy.is_shutdown():
                n=1
        except KeyboardInterrupt, e:
            print "exiting due to ", e

if __name__=="__main__":
    rospy.init_node('stl_generator',anonymous=True)
    generate_stl = stl_generator()
    generate_stl.generate_env()


