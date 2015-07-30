
import time
import openravepy
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import sys
sys.path.append('/stl_dae_files/')

def main(env,options):
	obj = env.ReadTrimeshFile('Coil1.STL');
	env.SetViewer('qtcoin')
	env.Load('robots/barrettwam.robot.xml')
	robot = env.GetRobots()[0]
	with env:
	    T = robot.GetLinks()[1].GetTransform()
	    print 'This is transformation matrix : '
	    print T

#def run(args=None):
#	parser = OptionParser(description='This is interface for controlling the robot in openrave')
#	OpenRAVEGlobalArguments.addOptions(parser)
#	(options,leftargs) = parser.parse_args(args=args)
#	OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)	
#

if __name__=="__main__":
	parser = OptionParser(description='This is interface for controlling the robot in openrave')
	OpenRAVEGlobalArguments.addOptions(parser)
	(options,leftargs) = parser.parse_args(args=None)
	OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)	
