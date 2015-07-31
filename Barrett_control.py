import threading
from openravepy import *
import rospy


def main():
	env = Environment()
	env.Load('robots/barrettwam.robot.xml')
	env.SetViewer('qtcoin')
	robot = env.GetRobots()[0]
	obj = env.ReadKinBodyXMLFile('Coil1.STL')
	env.Add(obj)
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
