#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from amsagv_msgs.msg import TagStamped
import subprocess
import rospkg
import os



rp = rospkg.RosPack()
path = os.path.join(rp.get_path('amsagv'), 'data', 'complete.oga')

def handleTag(msg):
  print('Tag ID: {}'.format(msg.tag.id))
  subprocess.call(['paplay', path])



try:
  rospy.init_node('tag_beep')
    
  # Tag subscriber
  sub = rospy.Subscriber('tag', TagStamped, handleTag)
  
  rospy.spin()
except KeyboardInterrupt:
  pass
