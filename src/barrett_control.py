#!/usr/bin/env python
import threading
from openravepy import *
import rospy
import numpy as np
import sys
import rospkg
path = rospkg.RosPack().get_path('barrett_hand_control')


def main():
	env = Environment()
        env.Load(path+'/src/barrett_wam.dae')
	env.SetViewer('qtcoin')
        robot = env.GetRobots()[0]
        obj = env.ReadKinBodyXMLFile(path+'/src/stl_files/Coil1.STL',{'scalegeometry':'0.001 0.001 0.001'})
        env.Add(obj)
        try:
                T_robot = robot.GetDOFValues()
                Trans_matrix = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,2.5],[0,0,0,1]])
                obj.SetTransform(Trans_matrix)
                while True:
                    n=1
	except KeyboardInterrupt, e: 
                print e
	finally:
		print 'exiting'
		env.Destroy()

if __name__=="__main__":
    main()
