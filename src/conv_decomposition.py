#!/usr/bin/env python
import threading
from openravepy import *
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import sys
import rospkg
import time


class control(object):
    def __init__(self):
        self.path = rospkg.RosPack().get_path('barrett_hand_control')
	self.env = None
        self.flag = 0
        self.env = Environment()
        self.env.Load(self.path+'/src/barrett_wam.dae')
        self.robot = self.env.GetRobots()[0]
        self.env.SetViewer('qtcoin')

    def generate_environment(self):
        try:
            self.hand = self.env.ReadRobotXMLFile(self.path+'/src/bhand.dae')
            self.wam = self.env.ReadRobotXMLFile(self.path+'/src/wam.dae')
#            self.robot.SetDOFValues([8.29153121e-01, -1.46121848e+00, 2.03694558e+00, 1.35058486e+00, -9.14774418e-01, 5.70182264e-01, 2.35470676e+00, 2.22044605e-16, 2.22044605e-16, 1.56904329e-02, 1.49526310e+00, 4.98535573e-01, 1.56904329e-02, 1.45774198e+00, 4.86576647e-01, 1.29563081e+00, 5.63570201e-01])
            self.robot.SetDOFValues([0.81366723775863636, -1.5860003709793091, 2.0457030773162836, 1.2624661684036254, -0.89291913509368837, 0.62145200967788694, 2.4271213531494147, 3.1086244689504381e-16, 3.1086244689504381e-16, 0.015690432861447334, 1.4682650566101076, 0.48958203792572014, 0.015690432861447424, 1.4379965782165527, 0.47997848391532927, 1.3054973840713502, 0.53829675912857122])
            self.env.Add(self.hand)
            self.env.Add(self.wam)
            self.obj = self.env.ReadKinBodyXMLFile(self.path+'/src/stl_files/CerealBox.STL',{'scalegeometry':'0.001 0.001 0.001'})
            self.env.Add(self.obj)
            self.Table = self.env.ReadKinBodyXMLFile('data/table.kinbody.xml')
            self.env.Add(self.Table)
            self.Table.SetTransform([[0.0007963267271406949, -0.9999997019767761, 0.0, -0.8919000029563904], [0.9999997019767761, 0.0007963267271406949, 0.0, 0.3614000082015991], [-0.0, 0.0, 1.0, 1.0058000087738037], [0.0, 0.0, 0.0, 0.0]])
            self.Trans_matrix = np.array([[-1.0, 3.140000104904175, -3.140000104904175, -0.71670001745224],[ 0.0, 0.0, -0.8199999928474426, -0.11140000075101852], [0.0, 0.0, 0.0, 1.0211999416351318], [0.0, 0.0, 0.0, 0.0]])
            self.obj.SetTransform(self.Trans_matrix)
            self.T_robot = self.robot.GetLinkTransformations()[9:23]
            self.hand.SetLinkTransformations(self.T_robot)
            self.T_wam = self.robot.GetLinkTransformations()[0:9]
            self.T_wam.append(self.robot.GetLinkTransformations()[-1])
            self.wam.SetLinkTransformations(self.T_wam)
            while not rospy.is_shutdown():
                    N=1

        except KeyboardInterrupt, e:
            print e
        finally:
            print 'exiting'
            self.env.Destroy()

    def updater(self,slider_values):
        self.flag = slider_values.data[17]
        if self.flag == 1:
            self.T_robot = self.robot.GetLinkTransformations()[9:23]
            self.hand.SetLinkTransformations(self.T_robot)
            self.T_wam = self.robot.GetLinkTransformations()[0:9]
            self.T_wam.append(self.robot.GetLinkTransformations()[-1])
            self.wam.SetLinkTransformations(self.T_wam)
        self.robot.SetDOFValues(slider_values.data[0:17])
    
    def updater_part(self,transform_values):
        transform = [[transform_values.data[0],transform_values.data[1],transform_values.data[2],transform_values.data[3]],[transform_values.data[4],transform_values.data[5],transform_values.data[6],transform_values.data[7]],[transform_values.data[8],transform_values.data[9],transform_values.data[10],transform_values.data[11]],[transform_values.data[12],transform_values.data[13],transform_values.data[14],transform_values.data[15]]]

        self.obj.SetTransform(transform)

    def convex_decomposition(self,points_mat):
        self.cdmodel = databases.convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.ab = self.robot.ComputeAABB()
        self.samplingdelta = numpy.linalg.norm(self.ab.extents())/30.0
        self.boxmin = self.ab.pos()-self.ab.extents()
        self.boxmax = self.ab.pos()+self.ab.extents()
        self.X,self.Y,self.Z = numpy.mgrid[self.boxmin[0]:self.boxmax[0]:self.samplingdelta,self.boxmin[1]:self.boxmax[1]:self.samplingdelta,self.boxmin[2]:self.boxmax[2]:self.samplingdelta]
        self.points = numpy.c_[self.X.flat,self.Y.flat,self.Z.flat]
        self.inside = self.cdmodel.testPointsInside(self.points)
        self.plottedpoints = self.points[numpy.flatnonzero(self.inside),:]
#        self.plottedpoints[:,1] += self.ab.extents()[1]*2
        self.points_mat_length = len(points_mat)
        print "running"
        self.points = numpy.reshape(points_mat,(self.points_mat_length/3,3))
        self.plot = self.env.plot3(self.plottedpoints,2)
        for i in range(self.points_mat_length/3):
            try:
                self.plottedpoints = np.delete(self.plottedpoints,self.points[i],axis=0)
            except ValueError:
                pass

        

def main():
    ctrl = control()
    rospy.init_node('control',anonymous = True)
    ctrl.sub = rospy.Subscriber("control_slider_values", Float32MultiArray, ctrl.updater)
    ctrl.sub_trans = rospy.Subscriber("slider_for_transformation",Float32MultiArray, ctrl.updater_part)
    #ctrl.sub_cd = rospy.Subscriber("points",Float32MultiArray,ctrl.convex_decomposition)
    ctrl.convex_decomposition(numpy.array([1,2,3]))
    ctrl.generate_environment()

if __name__=="__main__":
    main()
