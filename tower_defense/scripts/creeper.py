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
#the path coords should only be used by this node, but might be useful for something
#TODO: test this to make sure it actually works
def MakePathService(req):
	global path 

	p     = req.point_cloud
	start = req.start
	end   = req.end

	return MakePathServiceResponse(path)

#every time this is called the creepers will all move forward one step
#create_new = True -> put a new creeper at the start point
#so call this 20 times to make 20 creepers
#you'll probably want to stagger the calls to this so they don't overlap
#TODO: test this to make sure it actually works
def MoveCreepersService(req):
	global creepers

	create_new        = req.create_new
	current_locations = []
	reached_end       = False

	if create_new:
		new_creeper = [[creeper_health, -1]]
		creepers    = new_creeper + creepers

	for creeper in creepers:
		if not creeper[1] == len(path) - 1:
			creeper[1] += 1
			current_locations.append(path[creeper[1]])
		else:
			reached_end = True

	#if a creeper reached the end, delete it from the list of creepers
	#because of the way creepers are created, the last/closest to the end creeper
	#is always at the end of the list
	if reached_end:
		creepers = creepers[:-1]

	return MoveCreepersSrvResponse(current_locations, reached_end)

#TODO: test this to make sure it actually works
def HurtCreeperService(req):
	global creepers

	damage   = req.damage
	location = req.location

	for i in range(len(creepers)):
		if path[creepers[i][1]] == location:
			creepers[i][0] -= damage

			if creepers[i][0] <= 0:
				if i == len(creepers):
					creepers = creepers[:i]
				else:
					creepers = creepers[:i] + creepers[i+1:]

			return HurtCreeperSrvResponse([path[c[1]] for c in creepers])

	#if you get here that means that the location given wasn't the location
	#of any of the creeps
	assert False

if __name__ == "__main__":
  rospy.init_node('creepers')

  s1 = rospy.Service('/tower_defense/MakePath', \
                     MakePathSrv, \
                     MakePathService)

  s2 = rospy.Service('/tower_defense/MoveCreepers', \
                     MoveCreepersSrv, \
                     MoveCreepersService)

  s3 = rospy.Service('/tower_defense/HurtCreeper', \
                     HurtCreeperSrv, \
                     HurtCreeperService)

  print "creeper node established"

  rospy.spin()