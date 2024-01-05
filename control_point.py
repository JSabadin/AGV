#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped



try:
  rospy.init_node('control_line')
  
  # Velocity commands publisher.
  pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  
  # TF buffer and listener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  # Velocity commands message
  msgCmdVel = Twist()

  rate = rospy.Rate(50)
  while not rospy.is_shutdown():
    t = rospy.Time.now()
    
    try:
      trans = tfBuffer.lookup_transform('world', 'agv', rospy.Time())
      x, y, phi = ams.msgToPose(trans)
    except tf2_ros.TransformException:
      pass
      
    vs, ws = 0.0, 0.0
  
    msgCmdVel.linear.x = vs
    msgCmdVel.angular.z = ws
    # Publish velocity commands
    pubCmdVel.publish(msgCmdVel)
    
    rate.sleep()
except KeyboardInterrupt:
  pass
