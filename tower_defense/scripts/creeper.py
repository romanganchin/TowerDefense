#!/usr/bin/python

"""
How to use this node:
1) Call MakePathService to create the path for the creeps
2) When/if you want to damage a creep, use HurtCreeperService
3) When/if you want the creeps to move, call MoveCreeperService
3a) if you want to create creeps, call MoveCreeperService with create_new = True
4) Repeat 2 and 3 every timestep in whatever order makes the most sense
"""

import numpy as np
import random
import rospy
import roslib; roslib.load_manifest('tower_defense')
from geometry_msgs.msg import *
from tower_defense.srv import *

creeper_radius = 0.0002
path_step_size = 0.001
creeper_health = 10
creepers       = [] #each creeper should be [health, index of location in path]
path           = []
min_value      = -4
max_value      = 4
x_min = -0.745
x_max = 0.77
y_max = 0.6
y_min = -0.44

# Return a random value between min and max.
def RandomValue(min_value, max_value):
  return np.random.uniform(min_value, max_value)


class RRTNode():
	location = Point32()
	parent   = -1

def RandomConfig(goal):
	# global min_value
	# global max_value
	global x_min
	global x_max
	global y_min
	global y_max

	q_rand = Point32()
	if RandomValue(0, 1) <= 0.05:
		q_rand.x = goal.x
		q_rand.y = goal.y
	else:
		q_rand.x = np.random.uniform(x_min, x_max)
		q_rand.y = np.random.uniform(y_min, y_max)
	return q_rand

def ExtendNode(P, q):
	delta_q = 0.06

	q_new   = Point32()
	goal    = np.array([q.x, q.y])
	points  = np.array([[bad_point.x, bad_point.y] for bad_point in P])
	lengths = [np.linalg.norm(point-goal) for point in points]

	q_near_index = np.argmin(lengths)

	start = points[q_near_index]
	v     = goal - start
	if v.dot(v) > delta_q:
		v       /= v.dot(v)
		v       *= delta_q
		new_goal = start + v
	else:
		new_goal = goal

	q_new.x  = new_goal[0]
	q_new.y  = new_goal[1]

	return q_near_index, q_new

def CheckExtension(point_cloud, r, current, desired):
	V     = np.array([desired.x - current.x, desired.y - current.y])
	V_hat = V / V.dot(V)

	for point in point_cloud:
		P = np.array([point.x, point.y])
		n = np.array([-P[1], P[0]])

		projection = V + ((P-V).dot(V_hat)) * V_hat

		if (np.sqrt(projection.dot(projection)) <= r):
			print "(" + str(current.x) + ", " + str(current.y) + ") to (" + str(desired.x) + ", " + str(desired.y) + ") collided with (" + str(point.x) + ", " + str(point.y) + ")"
			return False

	return True

#returns a list of points, which are the steps the creepers take at each timestep
#the path coords should only be used by this node, but might be useful for something
#TODO: test this to make sure it actually works
def MakePathService(req):
	print "starting path service..."
	global path 
	global path_step_size
	global creeper_radius

	p     = req.point_cloud
	start = req.start
	goal  = req.end

	first_rrt = RRTNode()
  	first_rrt.location = start
  	first_rrt.parent   = -1
  	rrt = [first_rrt]

	current_point = start

	while np.sqrt((current_point.x - goal.x)**2 + (current_point.y - goal.y)**2) > 0.1:
		print "looking at a point..."
		raw_points  = [rr.location for rr in rrt]
		q_rand      = RandomConfig(goal)
		q_i, q_new  = ExtendNode(raw_points, q_rand)
		q_near      = raw_points[q_i]

		while not CheckExtension(p, creeper_radius, q_near, q_new):
			print "finding some new points..."
			print q_new
			q_rand      = RandomConfig(goal)
			q_i, q_new  = ExtendNode(raw_points, q_rand)
			q_near      = raw_points[q_i]

		current_rrt          = RRTNode()
		current_rrt.location = q_new
		current_rrt.parent   = q_i

		rrt.append(current_rrt)
		current_point = q_new

	rrt.insert(0, rrt[-1])
	rrt            = rrt[:-1]
	rrt[0].parent += 1

	for rr in rrt[2:]:
		rr.parent += 1

	i        = 0
	raw_path = []
	while not i == -1:
		raw_path.append(rrt[i].location)
		i = rrt[i].parent

	raw_path = raw_path[::-1]
	path     = []
	for i in range(len(raw_path)-1):
		path.append(raw_path[i])

		current_start = np.array([raw_path[i].x, raw_path[i].y])
		current_end   = np.array([raw_path[i+1].x, raw_path[i+1].y])
		path_vector   = current_end - current_start
		hat_vector    = path_vector / path_vector.dot(path_vector)
		current_len   = path_step_size
		current_max   = path_vector.dot(path_vector)

		while current_len < current_max:
			new_point = current_start + hat_vector * current_len
			p         = Point32()
			p.x       = new_point[0]
			p.y       = new_point[1]

			path.append(p)
			current_len += path_step_size


	print "here are the points"
	for p in path:
		print p

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

	l_to_d = {l: d for l, d in zip(req.location, req.damage)}

	for i in range(len(creepers)):
		if path[creepers[i][1]] in l_to_d:
			creepers[i][0] -= l_to_d[path[creepers[i][1]]]

	creepers = [c for c in creepers if c[0] > 0]

	#if you get here that means that the location given wasn't the location
	#of any of the creeps
	return HurtCreeperSrvResponse([path[c[1]] for c in creepers])

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