#!/usr/bin/python

"""
How to use this node:
1) Call MakePathService to create the path for the creeps
2) When/if you want to damage a creep, use HurtCreeperService
3) When/if you want the creeps to move, call MoveCreeperService
3a) if you want to create creeps, call MoveCreeperService with create_new = True
4) Repeat 2 and 3 every timestep in whatever order makes the most sense
"""

import numpy
import random
import rospy
import roslib; roslib.load_manifest('tower_defense')
from geometry_msgs.msg import *
from tower_defense.srv import *

creeper_radius = 1
path_step_size = 0.01
creeper_health = 10
creepers       = [] #each creeper should be [health, index of location in path]
path           = []
min_value      = -4
max_value      = 4

class RRTNode():
	location = Point32()
	parent   = -1

def RandomConfig(goal):
	global min_value
	global max_value

	q_rand = Point()
	if RandomValue(0, 1) <= 0.05:
		q_rand.x = goal.x
		q_rand.y = goal.y
	else:
		q_rand.x = np.random.uniform(min_value, max_value)
		q_rand.y = np.random.uniform(min_value, max_value)
	return q_rand

def ExtendNode(P, q):
	delta_q = 0.5

	q_new   = Point()
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

def GetFreePathService(point_cloud, r, current, desired):
	# Laser scan.
	laser_scan = data.laser_scan
	# Desired velocity vector.
	V = np.array([desired.x, desired.y])
	angle = data.laser_scan.angle_min
	free_path_length = float('inf')
	is_obstacle = False

	for magnitude in data.laser_scan.ranges:
	for pointy_p in point_cloud
		# P = np.array([np.cos(angle), np.sin(angle)]) * magnitude
		P = np.array([P.x, P.y])
		n = np.array([-P[1], P[0]])

		# angle += data.laser_scan.angle_increment

		projection = (P.dot(V) / V.dot(V)) * V
		delta      = np.sqrt(r**2 - P.dot(n)**2)
		distance   = np.sqrt(projection.dot(projection)) - delta

		n_projection = (P.dot(n) / n.dot(n)) * n

		if (np.sqrt(n_projection.dot(n_projection)) <= r):
			is_obstacle = True
		if distance < free_path_length:
			free_path_length = distance

#returns a list of points, which are the steps the creepers take at each timestep
#the path coords should only be used by this node, but might be useful for something
#TODO: test this to make sure it actually works
def MakePathService(req):
	global path 
	global path_step_size
	global creeper_radius

	p     = req.point_cloud
	start = req.start
	goal  = req.end

	current_point = first_point

	while np.sqrt((current_point.x - goal.x)**2 + (current_point.y - goal.y)**2) > 0.1:
		raw_points  = [rr.location for rr in rrt]
		q_rand      = RandomConfig(goal)
		q_i, q_new  = ExtendNode(raw_points, q_rand)
		q_near      = raw_points[q_i]

		while not CheckExtension(q_near, q_new):
			q_rand      = RandomConfig(goal)
			q_i, q_new  = ExtendNode(raw_points, q_rand)
			q_near      = raw_points[q_i]

		current_rrt          = RRTNode()
		current_rrt.location = q_new
		current_rrt.parent   = q_i

		rrt.append(current_rrt)
		current_point = q_new

	rrt.insert(0, rrt[-1])
	rrt = rrt[:-1]
	rrt[0].parent += 1

	for rr in rrt[2:]:
		rr.parent += 1

	i = 0
	path = []
	while not i == -1:
		path.append(rrt[i].location)
		plan_marker_.points.append(rrt[i].location)
		i = rrt[i].parent
		plan_marker_.points.append(rrt[i].location)

	plan_marker_.points = plan_marker_.points[:-2]
	markers.markers.append(plan_marker_)
	markers_publisher_.publish(markers)

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