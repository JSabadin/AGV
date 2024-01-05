#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from amsagv_msgs.msg import TagsStamped, ActionsStamped, TagStamped, ActionStamped
from PathPlanning import PathPlanning
from itertools import tee



def handleStartGoal(msg):
  try:
    path = pp.findPath(msg.tags[0].tag.id, msg.tags[-1].tag.id)
    patha, pathb = tee(path)

    msgPathTags = TagsStamped()
    for x in patha:
      pathTag = TagStamped()
      pathTag.tag.id = x
      msgPathTags.tags.append(pathTag)
    pubPathTags.publish(msgPathTags)

    actions = pp.generateActions(list(pathb))
    msgPathActions = ActionsStamped()
    for x in actions:
      pathAction = ActionStamped()
      pathAction.action.name = x[0]
      pathAction.action.id = x[1]
      pathAction.action.distance = x[2]
      msgPathActions.actions.append(pathAction)
    pubPathActions.publish(msgPathActions)
  except AttributeError:
    print('Error! Something went wrong :(')



try:
  rospy.init_node('path_planning')

  pp = PathPlanning()

  # Publisher of the path, which is given as a list of tag IDs
  pubPathTags = rospy.Publisher('path_tags', TagsStamped, queue_size=10, latch=True)
  # Publisher of the actions
  pubPathActions = rospy.Publisher('path_actions', ActionsStamped, queue_size=10, latch=True)
  # Subscriber to the start and goal tag ID
  subStartGoal = rospy.Subscriber('start_goal_tag', TagsStamped, handleStartGoal)

  rospy.spin()
except KeyboardInterrupt:
  pass
