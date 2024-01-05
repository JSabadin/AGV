#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from amsagv_msgs.msg import TagStamped, TagsStamped
from functools import partial

from graph_gen import findClosestNode



class TagsConverters(object):
  def __init__(self):        
    self._tfBuffer = tf2_ros.Buffer()
    self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
  
    # Name of the world frame
    self._worldFrameId = rospy.get_param('~world_frame_id', 'world')
    # Names of the robot
    self._robotNames = rospy.get_param('~robot_names', 'robot').split(' ')
    # Names of the robot frames
    self._robotFrameIds = rospy.get_param('~robot_frame_ids', 'robot').split(' ')

    self._pubStartGoal = {}
    self._subGoal = {}
    self._subGoalTag = {}
    self._lastStamp = {}

    for name, frame in zip(self._robotNames, self._robotFrameIds):
      # Publisher of the start and goal tag ID (the two values are separated by a comma). The message is published when a new goal is set and the start node is known.
      self._pubStartGoal[frame] = rospy.Publisher('{}/start_goal_tag'.format(name), TagsStamped, queue_size=10)
      # Subscriber to the goal pose.
      self._subGoal[frame] = rospy.Subscriber('{}/goal'.format(name), PoseStamped, partial(self._handleGoal, frame=frame))
      # Subscriber to the goal tag.
      self._subGoalTag[frame] = rospy.Subscriber('{}/goal_tag'.format(name), TagStamped, partial(self._handleGoalTag, frame=frame))
      self._lastStamp[frame] = None

  def _handleGoal(self, msg, frame=None):
    p = (msg.pose.position.x, msg.pose.position.y)
    best = findClosestNode(p)
    if best is not None:
      startTag = self._getStartTag(frame)
      if startTag is not None:
        self._publishStartGoal[frame](startTag, best[0])
            
  def _handleGoalTag(self, msg, frame=None):
    goalTag = msg.tag.id
    startTag = self._getStartTag(frame)
    if startTag is not None:
      self._publishStartGoal(startTag, goalTag, frame=frame)
      
  def _publishStartGoal(self, startTag, goalTag, frame=None):
    msg = TagsStamped()

    tag = TagStamped()
    tag.tag.id = startTag
    msg.tags.append(tag)

    tag = TagStamped()
    tag.tag.id = goalTag
    msg.tags.append(tag)

    self._pubStartGoal[frame].publish(msg)
      
  def _getStartTag(self, frame=None):
    startTag = None
    try:
      trans = self._tfBuffer.lookup_transform(self._worldFrameId, frame, rospy.Time())
      if self._lastStamp[frame] != trans:
        ps = (trans.transform.translation.x, trans.transform.translation.y)
        best = findClosestNode(ps)
        if best is not None:
          startTag = best[0]
        self._lastStamp[frame] = trans.header.stamp
    except tf2_ros.TransformException:
      pass
    
    return startTag



if __name__ == '__main__':
  rospy.init_node('tags_converters')
  tagsConverters = TagsConverters()
  rospy.spin()
