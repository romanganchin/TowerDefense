#!/usr/bin/python

import numpy
import random
import rospy
import roslib; roslib.load_manifest('tower_defense')
from geometry_msgs.msg import *
# from tower_defense.msg import *
from tower_defense.srv import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

creeper_radius = 1
path_step_size = 0.01
creeper_health = 10
creepers       = [] #each creeper should be [health, index of location in path]
path           = []

#returns a list of points, which are the steps the creepers take at each timestep
#the list should only be used by this node, but might be otherwise useful
def MakePathService(req):
	p = req.point_cloud
	pass

#create_new = True -> put a new creeper at the start point
#so call this 20 times to make 20 creepers
#you'll probably want to stagger the calls to this so they don't overlap
def MoveCreepersService(req):
	create_new = req.create_new


if __name__ == "__main__":
  rospy.init_node('creepers')

  s1 = rospy.Service('/TowerDefense/MakePath', \
                     MakePathSrv, \
                     MakePathService)

  s2 = rospy.Service('/TowerDefense/MoveCreepers', \
                     MoveCreepersSrv, \
                     MoveCreepersService)