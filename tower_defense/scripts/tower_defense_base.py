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

# Publisher for marker messages.
markers_publisher_ = None

# Markers for visualization.
vertices_marker_ = Marker()
qrand_marker_ = Marker()
edges_marker_ = Marker()
map_marker_ = Marker()
plan_marker_ = Marker()

# Return a random value between min and max.
def RandomValue(min_value, max_value):
  return uniform(min_value, max_value)

def InitMarkers():
  global vertices_marker_
  global qrand_marker_
  global edges_marker_
  global map_marker_
  global plan_marker_

  vertices_marker_.header.frame_id = "map"
  vertices_marker_.id = 1
  vertices_marker_.type = Marker.POINTS
  vertices_marker_.action = Marker.MODIFY
  vertices_marker_.scale.x = 0.2
  vertices_marker_.scale.y = 0.2
  vertices_marker_.color.a = 1.0
  vertices_marker_.color.r = 0.0
  vertices_marker_.color.g = 0.0
  vertices_marker_.color.b = 1.0

  qrand_marker_.header.frame_id = "map"
  qrand_marker_.id = 2
  qrand_marker_.type = Marker.POINTS
  qrand_marker_.action = Marker.MODIFY
  qrand_marker_.scale.x = 0.2
  qrand_marker_.scale.y = 0.2
  qrand_marker_.color.a = 1.0
  qrand_marker_.color.r = 1.0
  qrand_marker_.color.g = 0.0
  qrand_marker_.color.b = 0.0

  edges_marker_.header.frame_id = "map"
  edges_marker_.id = 3
  edges_marker_.type = Marker.LINE_LIST
  edges_marker_.action = Marker.MODIFY
  edges_marker_.scale.x = 0.05
  edges_marker_.scale.y = 0.05
  edges_marker_.color.a = 1.0
  edges_marker_.color.r = 0.0
  edges_marker_.color.g = 1.0
  edges_marker_.color.b = 0.0

  map_marker_.header.frame_id = "map"
  map_marker_.id = 4
  map_marker_.type = Marker.LINE_LIST
  map_marker_.action = Marker.MODIFY
  map_marker_.scale.x = 0.05
  map_marker_.scale.y = 0.05
  map_marker_.color.a = 1.0
  map_marker_.color.r = 1.0
  map_marker_.color.g = 1.0
  map_marker_.color.b = 1.0

  plan_marker_.header.frame_id = "map"
  plan_marker_.id = 5
  plan_marker_.type = Marker.LINE_LIST
  plan_marker_.action = Marker.MODIFY
  plan_marker_.scale.x = 0.05
  plan_marker_.scale.y = 0.05
  plan_marker_.color.a = 1.0
  plan_marker_.color.r = 1.0
  plan_marker_.color.g = 0.0
  plan_marker_.color.b = 0.0


def RandomConfigService(req):
  q_rand = Point()
  return RandomConfigSrvResponse(q_rand)

def ExtendNodeService(req):
  q_near_index = -1
  q_new = Point()
  return ExtendNodeSrvResponse(q_near_index, q_new)

def CheckExtensionService(req):
  valid = True
  return CheckExtensionSrvResponse(valid)

def BuildRRTService(req):
  rrt = []
  global vertices_marker_
  global qrand_marker_
  global edges_marker_
  global map_marker_
  global plan_marker_
  markers = MarkerArray()
  markers.markers.append(vertices_marker_)
  markers.markers.append(qrand_marker_)
  markers.markers.append(edges_marker_)
  markers.markers.append(map_marker_)
  markers.markers.append(plan_marker_)
  markers_publisher_.publish(markers)

  return BuildRRTSrvResponse(rrt)

def RRTPlanService(req):
  plan = []
  return RRTPlanSrvResponse(plan)

def RRTPlanBonusService(req):
  return RRTPlanSrvResponse(plan)

if __name__ == "__main__":
  InitMarkers()
  rospy.init_node('compsci403_assignment6')

  markers_publisher_ = rospy.Publisher('/COMPSCI403/RRT_Display', \
                                       MarkerArray, \
                                       queue_size=10)

  s1 = rospy.Service('/COMPSCI403/RandomConfig', \
                     RandomConfigSrv, \
                     RandomConfigService)

  s2 = rospy.Service('/COMPSCI403/ExtendNode', \
                     ExtendNodeSrv, \
                     ExtendNodeService)

  s3 = rospy.Service('/COMPSCI403/CheckExtension', \
                     CheckExtensionSrv, \
                     CheckExtensionService)

  s4 = rospy.Service('/COMPSCI403/BuildRRT', \
                     BuildRRTSrv, \
                     BuildRRTService)

  s5 = rospy.Service('/COMPSCI403/RRTPlan', \
                     RRTPlanSrv, \
                     RRTPlanService)

  s6 = rospy.Service('/COMPSCI403/RRTPlanBonus', \
                     RRTPlanSrv, \
                     RRTPlanService)

  rospy.spin()

