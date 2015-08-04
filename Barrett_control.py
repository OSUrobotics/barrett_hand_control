import threading
from openravepy import *
import rospy


def main():
	env = Environment()
        env.Load('robots/barrettwam.robot.xml')
	env.SetViewer('qtcoin')
        obj = env.ReadKinBodyXMLFile('stl_files/Coil1.STL',{'scalegeometry':'0.001 0.001 0.001'})
	robot = env.GetRobots()[0]
        env.Add(obj)
        T_robot = robot.GetLinkTransformations()
        print (T_robot)
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
