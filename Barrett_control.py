import threading
from openravepy import *
import rospy
import numpy as np
import sys


def main():
	env = Environment()
        env.Load('barrett_wam.dae')
	env.SetViewer('qtcoin')
	robot = env.GetRobots()[0]
        obj = env.ReadKinBodyXMLFile('stl_files/Coil1.STL',{'scalegeometry':'0.001 0.001 0.001'})
        env.Add(obj)
        T_robot = robot.GetDOFValues()
        Trans_matrix = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,2.5],[0,0,0,1]])
        obj.SetTransform(Trans_matrix)
#        T_robot = robot.SetDOFValues([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
	try:
		while True:
			n=1 
	except KeyboardInterrupt, e: 
                print e
	finally:
		print 'exiting'
		env.Destroy()

if __name__=="__main__":
	main()
