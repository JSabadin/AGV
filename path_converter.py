#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from amsagv_msgs.msg import TagsStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from graph import Graph



def handlePath(msg):
  if len(msg.tags):
    path = (x.tag.id for x in msg.tags)
    p = g.getPath(path)
    msgPath = Path()
    msgPath.header.frame_id = paramWorldFrameId
    msgPath.header.stamp = rospy.Time.now()
    for i in range(p.shape[1]):
      pose = PoseStamped()
      pose.pose.position.x = p[0,i]
      pose.pose.position.y = p[1,i]
      msgPath.poses.append(pose)
    pubPath.publish(msgPath)



try:
  rospy.init_node('path_converter')
  # Name of the world frame
  paramWorldFrameId = rospy.get_param('~world_frame_id', 'world')

  g = Graph()

  lastStamp = None

  # Publisher of the path
  pubPath = rospy.Publisher('path', Path, queue_size=10, latched=True)
  # Subscriber to the path, which is given as a list of tag IDs
  subPathTags = rospy.Subscriber('path_tags', TagsStamped, handlePath)

  rospy.spin()
except KeyboardInterrupt:
  pass
