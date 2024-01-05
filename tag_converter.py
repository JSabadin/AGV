#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from graph import Graph

class TagConverter(object):
  def __init__(self):
    self._graph = Graph()
    
    self._lastStamp = None
    
    self._tfBuffer = tf2_ros.Buffer()
    self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
  
    # Name of the world frame
    self._worldFrameId = rospy.get_param('~world_frame_id', 'world')
    # Name of the robot frame
    self._robotFrameId = rospy.get_param('~robot_frame_id', 'robot')
    # Publisher of the start and goal tag ID (the two values are separated by a comma). The message is published when a new goal is set and the start node is known.
    self._pubStartGoal = rospy.Publisher('start_goal_tag', String, queue_size=10)
    # Publisher of the path.
    self._pubPath = rospy.Publisher('path', Path, queue_size=10)
    # Subscriber to the path, which is given as a list of comma separated tag IDs.
    self._subPathTags = rospy.Subscriber('path_tags', String, self._handlePath)
    # Subscriber to the goal pose.
    self._subGoal = rospy.Subscriber('goal', PoseStamped, self._handleGoal)
    # Subscriber to the goal tag.
    self._subGoalTag = rospy.Subscriber('goal_tag', String, self._handleGoalTag)

  def _handlePath(self, msg):
    path = msg.data.split(',')
    try:
      path = [int(x) for x in path]
    except ValueError:
      return
    
    if len(path):
      p = self._graph.getPath(path)
      msgPath = Path()
      msgPath.header.frame_id = 'world'
      msgPath.header.stamp = rospy.Time.now()
      for i in range(p.shape[1]):
        pose = PoseStamped()
        pose.pose.position.x = p[0,i]
        pose.pose.position.y = p[1,i]
        msgPath.poses.append(pose)
      self._pubPath.publish(msgPath)
      
  def _handleGoal(self, msg):
    p = (msg.pose.position.x, msg.pose.position.y)
    best = self._graph.findClosestNode(p, self._graph.detTags)
    if best is not None:
      startTag = self._getStartTag()
      if startTag is not None:
        self._publishStartGoal(startTag, best[0])
            
  def _handleGoalTag(self, msg):
    try:
      goalTag = int(msg.data)
    except ValueError:
      return
      
    startTag = self._getStartTag()  
    if startTag is not None:
      self._publishStartGoal(startTag, goalTag)
      
  def _publishStartGoal(self, startTag, goalTag):
    msg = String()
    msg.data = '{},{}'.format(startTag, goalTag)
    self._pubStartGoal.publish(msg)
      
  def _getStartTag(self):
    startTag = None
    try:
      trans = self._tfBuffer.lookup_transform(self._worldFrameId, self._robotFrameId, rospy.Time())
      if self._lastStamp != trans:
        ps = (trans.transform.translation.x, trans.transform.translation.y)
        best = self._graph.findClosestNode(ps, self._graph.detTags)
        if best is not None:
          startTag = best[0]
        self._lastStamp = trans.header.stamp
    except tf2_ros.TransformException:
      pass
    
    return startTag

if __name__ == '__main__':
  rospy.init_node('tag_converter')
  tagConverter = TagConverter()
  rospy.spin()
