
import time
import openravepy
from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments
import sys
sys.path.append('/stl_dae_files/')


def main(env,options):
     env.Load('Coil1.STL')
     env.SetViewer('qtcoin')
     robot = env.GetRobots()[0]
     env.Load('robots/barrettwam.robot.xml')
     robot1 = env.GetRobots()[1]
     with env:
         T = robot.GetLinks()[1].GetTransform()
         T2 = robot1.GetLinks()[1].GetTransform()
         print 'This is transformation matrix : '
         print T, T2

def run(args=None):
	parser = OptionParser(description='This is interface for controlling the robot in openrave')
	OpenRAVEGlobalArguments.addOptions(parser)
	(options,leftargs) = parser.parse_args(args=args)
	OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options,main,defaultviewer=True)	

if __name__=="__main__":
	run()
