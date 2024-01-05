#!/usr/bin/python3
# -*- coding: utf-8 -*-
''' Display of the ground area

This program provides a simple user interface for visualization of the ground area.

Use left mouse click to define the goal tag. When a new goal is defined and the start tag is known, a message with the start and goal tag ID are published on the topic **start_goal_tag**. The start tag is defined based on the current pose of the robot frame with respect to the world frame. You can use right mouse click to manually define the start tag. Alternatively, the goal pose can also be set throught the topic **goal**.

When a new message on the topic **path_tags** is received, which contains a list of comma separated tags that represent the path, the path is drawn in the graph and a message with the path points is published on the topic **path**.

![graph-tags](img/graph-tags.png "Graph.")
'''
import sys
import matplotlib
matplotlib.use('Qt5Agg') # Make sure that we are using QT5
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from amsagv_msgs.msg import TagsStamped, TagStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from threading import Lock, Thread
from graph import Graph



class AmsEvent(object):
  def __init__(self, x, y, button=1):
    self.xdata = x
    self.ydata = y
    self.button = button



class AmsMapCanvas(FigureCanvas):
  def __init__(self, parent=None):
    self._mutex = Lock()

    self._graph = Graph()
    self._graph.createFigure(fig=Figure)

    self._goal = None
    self._start = None
    self._path = []
    self._startTag = None
    self._lastStamp = None

    FigureCanvas.__init__(self, self._graph.fig)
    self.setParent(parent)
    FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
    FigureCanvas.updateGeometry(self)

    self._graph.fig.canvas.setFocusPolicy( QtCore.Qt.ClickFocus )
    self._graph.fig.canvas.setFocus()
    self._graph.fig.canvas.mpl_connect('button_press_event', self._onButtonPress)

    self._tfBuffer = tf2_ros.Buffer()
    self._tfListener = tf2_ros.TransformListener(self._tfBuffer)

    # Name of the world frame
    self._worldFrameId = rospy.get_param('~world_frame_id', 'world')
    # Name of the robot frame
    self._robotFrameId = rospy.get_param('~robot_frame_id', 'robot')
    # Publisher of the start and goal tag ID (the two values are separated by a comma). The message is published when a new goal is set and the start node is known.
    self._pubStartGoal = rospy.Publisher('start_goal_tag', TagsStamped, queue_size=10)
    # Publisher of the path.
    self._pubPath = rospy.Publisher('path', Path, queue_size=10)
    # Subscriber to the path, which is given as a list of comma separated tag IDs.
    self._subPathTags = rospy.Subscriber('path_tags', TagsStamped, self._handlePath)
    # Subscriber to the goal pose.
    self._subGoal = rospy.Subscriber('goal', PoseStamped, self._handleGoal)

    self.updateCanvas()

  def _onButtonPress(self, event):
    p = (event.xdata, event.ydata)
    if None not in p:
      if event.button == 1:
        try:
          trans = self._tfBuffer.lookup_transform(self._worldFrameId, self._robotFrameId, rospy.Time())
          if self._lastStamp != trans:
            ps = (trans.transform.translation.x, trans.transform.translation.y)
            best = self._graph.findClosestNode(ps, self._graph.detTags)
            if best is not None:
              self._start = best[1]
              self._startTag = best[0]
            self._lastStamp = trans.header.stamp
        except tf2_ros.TransformException:
          pass

        best = self._graph.findClosestNode(p, self._graph.tags)
        if best is not None:
          self._goal = best[1]
          if self._startTag is not None:
            msg = TagsStamped()
            tag = TagStamped()
            tag.tag.id = self._startTag
            msg.tags.append(tag)
            tag = TagStamped()
            tag.tag.id = best[0]
            msg.tags.append(tag)
            self._pubStartGoal.publish(msg)
          self.updateCanvas()
      elif event.button == 3:
        best = self._graph.findClosestNode(p, self._graph.tags)
        if best is not None:
          self._start = best[1]
          self._startTag = best[0]
          self.updateCanvas()

  def updateCanvas(self):
    self._mutex.acquire()
    try:
      self._graph.draw(goal=self._goal, start=self._start, path=self._path)
      self.draw()
    finally:
      self._mutex.release()

  def _handlePath(self, msg):
    path = [x.tag.id for x in msg.tags]

    if len(path):
      self._path = path
      p = self._graph.getPath(self._path)
      msgPath = Path()
      msgPath.header.frame_id = 'world'
      msgPath.header.stamp = rospy.Time.now()
      for i in range(p.shape[1]):
        pose = PoseStamped()
        pose.pose.position.x = p[0,i]
        pose.pose.position.y = p[1,i]
        msgPath.poses.append(pose)
      self._pubPath.publish(msgPath)
      self.updateCanvas()

  def _handleGoal(self, msg):
    event = AmsEvent(msg.pose.position.x, msg.pose.position.y)
    self._onButtonPress(event)



class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.setWindowTitle('Display')

        self._fileMenu = QtWidgets.QMenu('&File', self)
        self._fileMenu.addAction('&Quit', self._onFileQuit, QtCore.Qt.CTRL + QtCore.Qt.Key_Q)
        self.menuBar().addMenu(self._fileMenu)

        self._main = QtWidgets.QWidget(self)

        layout = QtWidgets.QVBoxLayout(self._main)
        self.canvas = AmsMapCanvas(self._main)
        layout.addWidget(self.canvas)

        self._main.setFocus()
        self.setCentralWidget(self._main)

        self.statusBar().showMessage(":)", 2000)

    def _onFileQuit(self):
        self.close()

    def closeEvent(self, event):
        self._onFileQuit()



if __name__ == '__main__':
  try:
    rospy.init_node('display')
    Thread(target=rospy.spin, daemon=True).start()
    app = QtWidgets.QApplication(sys.argv)
    aw = ApplicationWindow()
    aw.show()
    app.exec_()
  except:
    pass
