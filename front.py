#!/usr/bin/python3
# -*- coding: utf-8 -*-
from ams import phiToQuaternionMsg
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from amsagv_msgs.msg import LineStamped



def handleLine(msg):
  trans.header.stamp = rospy.Time.now()
  trans.transform.rotation = phiToQuaternionMsg(msg.line.heading)
  tfBroadcaster.sendTransform(trans)

AGVS = ['anthony', 'goran', 'jaka', 'klemen', 'luka', 'matic']

try:
  rospy.init_node('front')
  
  ns = rospy.get_namespace().strip('/')
  x = AGVS.index(ns)
  
  trans = TransformStamped()
  
  trans.header.frame_id = 'front{}'.format(x)
  trans.child_frame_id = 'cart{}'.format(x)
  
  tfBroadcaster = tf2_ros.TransformBroadcaster()
  
  # Odometry subscriber
  subLine = rospy.Subscriber('line', LineStamped, handleLine)

  rospy.spin()
except KeyboardInterrupt:
  pass
