#!/usr/bin/python

import numpy
import random
import rospy
import roslib; roslib.load_manifest('compsci403_assignment6')
from geometry_msgs.msg import *
from compsci403_assignment6.msg import *
from compsci403_assignment6.srv import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

creeper_radius = 1
path_step_size = 0.01
creeper_health = 10
creepers       = []
path           = []

#returns a list of points, which are the steps the creepers take at each timestep
#the list should only be used by this service, but might be otherwise useful
def MakePathService(req):
	p = req.pointcloud
	pass

#create_new = True -> put a new creeper at the start point
#so call this 20 times to make 20 creepers
#you'll probably want to stagger the calls to this so they don't overlap
def MoveCreepers(req):
	create_new = req.create_new


if __name__ == "__main__":
  rospy.init_node('creepers')