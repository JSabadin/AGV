#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped
from math import pi, sin, cos, isnan
from world import MTAG
from amsagv_msgs.msg import ActionsStamped
from nav_msgs.msg import Odometry
from graph_gen import tagPoses
import numpy as np


# Define functions for control to reference position wrap_to_pi, deg2rad, control_to_pose
def wrap_to_pi(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

def deg2rad(degrees):
    return degrees * np.pi / 180

def rad2deg(radians):
    return radians * 180 / np.pi

def control_to_pose(q, goal):
    # Constants
    D = 0.1207
    Kv = 0.8; Kw = 9; Kg = 6
    r = 0.7

    d_err = np.linalg.norm(goal[:2] - q[:2])
    phi_t = np.arctan2(goal[1] - q[1], goal[0] - q[0])
    alpha = wrap_to_pi(phi_t - goal[2])
    beta = np.arctan(r / d_err)
    if alpha < 0:
        beta = -beta
        
    if abs(alpha) < abs(beta):
      phi_err = wrap_to_pi(phi_t - q[2] + alpha)
    else:
      phi_err = wrap_to_pi(phi_t - q[2] + beta)

    omega = Kw * phi_err
    v =  Kv * d_err

    gamma_reff = np.arctan2(omega * D, v)
    gamma_err = wrap_to_pi(gamma_reff - q[3])

    vs = min(np.sqrt((omega * D)**2 + v**2), 0.2)

    omegas = min(Kg * gamma_err, 13.95)

    return vs, omegas


# Global variables for control line 
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
  global x, y, phi
  lineLeft = msg.line.left 
  lineRight = msg.line.right
  fd = msg.line.distance
  
  if oldActions != actions:
    oldActions = actions
    pathIndex = 0
    tag = None
    accumulated_distances = None

  if accumulated_distances is None:
     accumulated_distances = fd

  # Define the constants
  Kpl = 2.5
  Kpr = 3.8
  Kps = 1
  # Define the  speed v_s. The 0.115 m/s is perfectly balanced as all things should be.
  v = 0.115

  if pathIndex <= len(actions) - 1:
    oldActions = actions 
    distance = actions[pathIndex].action.distance
    tagID = actions[pathIndex].action.id
    direction = actions[pathIndex].action.name
    
    if (actions[pathIndex - 1].action.id == 137):
      distance = 1.2
      v = 0.2
    if (actions[pathIndex - 1].action.id == 132):
      distance = 1.1
      v = 0.2
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

    elif direction == "straight":
      # Mid-empty space. We aim for the right line.
      if tagID in [142, 143]:
        x_goal142, y_goal142 =  (tagPoses[142][0:2])
        x_goal143, y_goal143 =  (tagPoses[143][0:2])
        if tagID == 142:
          # Constants are added since the odometry is not perfectly accurate
          x_goal = (x_goal142 + x_goal143)/2 + 0.2
          y_goal = (y_goal142 + y_goal143)/2 - 0.05
        else:
          # Constants are added since the odometry is not perfectly accurate
          x_goal = (x_goal142 + x_goal143)/2 + 0.1
          y_goal = (y_goal142 + y_goal143)/2 - 0.05  
        phi_goal = deg2rad(90)
        goal = np.array([x_goal, y_goal, phi_goal])
        q = [x, y, phi, gamma]
        v, w = control_to_pose(q, goal)
        # follow the line, when its detected
        if not isnan(lineRight):
            e_right = -0.5 - lineRight
            w = Kpr * (-e_right)
            # Slow down a bit
            v = 0.10
      else:
        # Normal straight direction
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

def handelEstim(msg):
  global x, y, phi
  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y
  phi = 2*np.arctan2(msg.pose.pose.orientation.z,msg.pose.pose.orientation.w) 

def handleOdometry(msg):
   global gamma
   gamma = msg.pose.pose.position.z

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
  # Localisation subscriber
  subEstim = rospy.Subscriber('estim', Odometry, handelEstim)
  # Odometry subscriber
  subOdom = rospy.Subscriber('odom', Odometry, handleOdometry)

  rospy.spin()
except KeyboardInterrupt:
  pass
