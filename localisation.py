#!/usr/bin/python3
# -*- coding: utf-8 -*-
from math import *
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import TagStamped
import numpy as np
from graph_gen import tagPoses
#from ams import wrapToPi
from world import MTAG
#import graph_gen as graph
#import tf.transformations as tft


def wrap_to_pi(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

def kalman_filter_prediction(q_prev, u, P_estimate, Q):  
    """
    Prediction of states and covariance matrix P for robot localization.
    
    Parameters:
    q_prev: Initial states [x, y, phi].
    u: Input signals [delta_d, gamma].
    P_estimate : Estimation of covariance matrix
    Q: Process noise covariance.

    Returns:
    Numpy array of updated state prediciton (q_next).
    """
    # System parameters
    D = 0.1207
    # Robot input
    delta_d, gamma = u
    # Robot state
    x_prev, y_prev, phi_prev = q_prev
    # Prediction
    x = x_prev + delta_d * np.cos(gamma) * np.cos(phi_prev)
    y = y_prev + delta_d * np.cos(gamma) * np.sin(phi_prev)
    phi = phi_prev + (delta_d * np.sin(gamma)) / D
    q_estimate = [x, y, phi]

    # Control input matrix (F)
    F = np.array([[np.cos(gamma) * np.cos(q_prev[2]), -delta_d * np.sin(gamma) * np.cos(q_prev[2])],
                  [np.sin(q_prev[2]) * np.cos(gamma), -delta_d * np.sin(gamma) * np.sin(q_prev[2])],
                  [np.sin(gamma) / D, delta_d * np.cos(gamma) / D]])
    
    # State transition matrix (A)
    A = np.array([[1, 0, -delta_d * np.sin(q_prev[2]) * np.cos(gamma)],
                  [0, 1, delta_d * np.cos(gamma) * np.cos(q_prev[2])],
                  [0, 0, 1]])
    
    # Prediction Step
    P_predict = A @ P_estimate @ A.T + F @ Q @ F.T  # Predicted covariance estimate

    return q_estimate, P_predict


def kalman_filter_correction(q_prediciton, P_prediction, z, R):
    """
    Kalman Filter correction for robot localization.
    
    Parameters:
    q_prediciton: Initial state estimate [x, y, phi].
    P_prediction: Initial covariance estimate.
    z: Measurement input [x_m, y_m, phi_m].
    R: Measurement noise covariance.

    Returns:
    Tuple of updated state estimate (q_next) and updated covariance estimate (P_next).
    """

    # Define the system parameters
    D = 0.1207
    M = 0.06  # Example value, adjust as necessary

    # Convert q_prediciton to numpy array for matrix operations
    q_prediciton = np.array(q_prediciton) 

    # Measurement matrix (C)
    C = np.array([[1, 0, -M * np.sin(q_prediciton[2]) ],
                  [0, 1, M * np.cos(q_prediciton[2])],
                  [0, 0, 1]])

    # Update Step
    S = C @ P_prediction @ C.T + R  # System uncertainty
    K = P_prediction @ C.T @ np.linalg.inv(S)  # Kalman Gain
    h = np.array([q_prediciton[0] + M * np.cos(q_prediciton[2]), q_prediciton[1] + M * np.sin(q_prediciton[2]), q_prediciton[2]])
    inovation = np.array(z) - h
    inovation[2] = wrap_to_pi(inovation[2])
    q_next = q_prediciton + K @ (inovation)  # Updated state estimate
    P_next = P_prediction - (K @ C @ P_prediction)  # Updated covariance estimate

    return q_next, P_next



class Localisation(object):
  def __init__(self):
    self._tfBroadcaster = tf2_ros.TransformBroadcaster()

    # Odometry subscriber
    self._subOdom = rospy.Subscriber('odom', Odometry, self._handleOdometry)
    # Tag subscriber
    self._subTag = rospy.Subscriber('tag', TagStamped, self._handleTag)

    self._pubEstim = rospy.Publisher('estim', Odometry, queue_size=10)

    self.x, self.y, self.phi = 0.0, 0.0, 0.0
    self.new_tag = None

    # Kalman Filter parameters
    self.q_estimate = [self.x, self.y, self.phi]  # Initial state estimate [x, y, phi]
    self.P_estimate = np.diag([1.0, 1.0, 0.1])  # Initial covariance estimate
    self.R = np.diag([0.001, 0.001, 0.001])  # Measurement noise covariance
    self.last_tag_detected = None

  def _handleTag(self, msg):
    self.new_tag = MTAG.get(msg.tag.id, None)
    print('Tag ID: {}'.format(msg.tag.id))

  def _handleOdometry(self, msg):
    # Check how to perform reading of deltaD and gamma
    deltaD = msg.twist.twist.linear.x 
    gamma = msg.pose.pose.position.z
    self.Q = np.diag([1* deltaD, 1 * deltaD]) # Process noise covariance
    u = [deltaD, gamma]

    self.q_estimate, self.P_estimate = kalman_filter_prediction(self.q_estimate, u, self.P_estimate, self.Q)
    # If we get the measuremant we correct the position
    if self.new_tag != self.last_tag_detected and self.new_tag in tagPoses:
        self.last_tag_detected = self.new_tag
        x_tag, y_tag =  tagPoses[self.new_tag][0:2]
        phi_tag = tagPoses[self.new_tag][2]
        z = [x_tag, y_tag, phi_tag]

        # Update Kalman Filter with new tag measurement
        self.q_estimate, self.P_estimate = kalman_filter_correction(self.q_estimate, self.P_estimate, z, self.R)

    self.x, self.y, self.phi = self.q_estimate


    print(np.diag(self.P_estimate))
    estim = Odometry()
    estim.header.frame_id = 'world'
    estim.child_frame_id = 'anthony/estim'
    estim.header.stamp = rospy.Time.now()
    estim.pose.pose.position.x = self.x
    estim.pose.pose.position.y = self.y
    estim.pose.pose.orientation.z = sin(self.phi/2.0)
    estim.pose.pose.orientation.w = cos(self.phi/2.0)
    estim.pose.covariance[0:2] = self.P_estimate[0][0:2]
    estim.pose.covariance[6:8] = self.P_estimate[1][0:2]
    estim.pose.covariance[35] =  self.P_estimate[2][2]
    estim.pose.covariance[5] = self.P_estimate[0][2]
    estim.pose.covariance[11] = self.P_estimate[1][2]
    estim.pose.covariance[30:32] = self.P_estimate[2][0:2]
    self._pubEstim.publish(estim)

    trans = TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'world'
    trans.child_frame_id = 'anthony/estimate'
    trans.transform.translation.x = self.x
    trans.transform.translation.y = self.y
    trans.transform.rotation.z = sin(self.phi/2.0)
    trans.transform.rotation.w = cos(self.phi/2.0)
    self._tfBroadcaster.sendTransform(trans)
    '''
    vecTodom2agv = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    vecQodom2agv = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    vecTworld2agv = np.array([self.x, self.y, 0.0])
    vecQworld2agv = np.array([0.0, 0.0, sin(self.phi/2.0), cos(self.phi/2.0)])
    vecQworld2odom = tft.quaternion_multiply(vecQworld2agv, tft.quaternion_conjugate(vecQodom2agv))
    matRworld2odom = tft.quaternion_matrix(vecQworld2odom)[0:3,0:3]
    vecTworld2odom = vecTworld2agv - matRworld2odom.dot(vecTodom2agv)
    msgQworld2odom = Quaternion(*vecQworld2odom)
    msgTworld2odom = Point(*vecTworld2odom)

    trans2 = TransformStamped()
    trans2.header.stamp = msg.header.stamp
    trans2.header.frame_id = 'world'
    trans2.child_frame_id = 'NS/odom'
    trans2.transform.translation = msgTworld2odom
    trans2.transform.rotation = msgQworld2odom

    transMsg = TransformStamped()
    transMsg.header = msg.header
    transMsg.child_frame_id = msg.child_frame_id
    transMsg.transform.translation = msg.pose.pose.position
    transMsg.transform.rotation = msg.pose.pose.orientation

    self._tfBroadcaster.sendTransform([trans2, transMsg])
    #'''



if __name__ == '__main__':
  try:
    rospy.init_node('localisation')

    localisation = Localisation()

    rospy.spin()
  except KeyboardInterrupt:
    pass
