#!/usr/bin/python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped



def handleInitialPose(msg):
  st.header = msg.header
  st.transform.translation = msg.pose.pose.position
  st.transform.rotation = msg.pose.pose.orientation

  broadcaster.sendTransform(st)



rospy.init_node('set_odom_frame')

ns = rospy.get_namespace().strip('/')
st = TransformStamped()
st.child_frame_id = '{}/odom'.format(ns)
broadcaster = tf2_ros.StaticTransformBroadcaster()

# Initial pose subscriber
subCmdVel = rospy.Subscriber('initial_pose', PoseWithCovarianceStamped, handleInitialPose)

rospy.spin()
