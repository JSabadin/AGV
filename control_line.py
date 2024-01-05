#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped
from math import pi, sin, cos, isnan
from world import MTAG
from amsagv_msgs.msg import ActionsStamped


pathIndex = 0
tag = None
accumulated_distances = None
oldActions = None


# Handle line sensor
def handleLine(msg):
  global tag
  global pathIndex
  global accumulated_distances
  global oldActions

  lineLeft = msg.line.left 
  lineRight = msg.line.right
  fd = msg.line.distance

  if accumulated_distances is None:
     accumulated_distances = fd

  # Define the constants
  v_max = 0.1
  Kpl = 2.5
  Kpr = 3.8
  Kps = 1
  # Define the  speed v_s
  v = 0.1

  if pathIndex <= len(actions) - 1:
    oldActions = actions 
    distance = actions[pathIndex].action.distance
    tagID = actions[pathIndex].action.id
    direction = actions[pathIndex].action.name
    print(tagID)
    if tagID < 21:
      if tagID == tag:
          pathIndex = pathIndex + 1
          accumulated_distances = fd
    else:
      if fd - accumulated_distances > distance:
          pathIndex = pathIndex + 1
          accumulated_distances = fd

    # Calculate the omega_s (angular velocity) to follow the left or the right line, or go straight
    if direction == "left":
        if not isnan(lineLeft):
            e_left = 0.5 - lineLeft
            w = Kpl * (-e_left)
        else:
            v, w = 0.0, 0.0

    elif direction == "right":
        if not isnan(lineRight):
            e_right = -0.5 - lineRight
            w = Kpr * (-e_right)  # Adjusting with Kpr
        else:
            v, w = 0.0, 0.0

    # elif direction == "straight":
    #     # To go straight, set angular velocity to 0
    #     w = 0.0
    elif direction == "straight":
      if not isnan(lineLeft) and not isnan(lineRight):
          e_straight = ((0.5 - lineLeft) + (-0.5 - lineRight)) / 2
          w = Kps * (-e_straight) 
      else:
          v, w = 0.0, 0.0

    else:
        # If smer is not defined or recognized, stop the cart
        v, w = 0.0, 0.0  
    
  # STOP if we arrive to final node
  else:
    e_straight = ((0.5 - lineLeft) + (-0.5 - lineRight)) / 2
    w = Kps * (-e_straight) 
    v = 0.0

  if oldActions != actions:
    oldActions = actions
    pathIndex = 0
    tag = None
    accumulated_distances = None



  # Velocity commands message
  msgCmdVel = Twist()
  msgCmdVel.linear.x = v
  msgCmdVel.angular.z = w
  # Publish velocity commands
  pubCmdVel.publish(msgCmdVel)


def handleTag(msg):
  global tag
  tag = MTAG.get(msg.tag.id, None)
  print('New tag: {} -> {}'.format(msg.tag.id, tag))


def handleActions(msg):
  global actions
  actions = msg.actions


try:
  rospy.init_node('control_line')
  
  # Velocity commands publisher.
  pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  # Line sensor subscriber
  subLine = rospy.Subscriber('line', LineStamped, handleLine)
  # Tag subscriber
  subTag = rospy.Subscriber('tag', TagStamped, handleTag)
  # Path and actions subscriber
  subActions = rospy.Subscriber('path_actions', ActionsStamped, handleActions)

  rospy.spin()
except KeyboardInterrupt:
  pass
