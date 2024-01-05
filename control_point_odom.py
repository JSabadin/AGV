#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

def wrap_to_pi(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

def deg2rad(degrees):
    return degrees * np.pi / 180

def rad2deg(radians):
    return radians * 180 / np.pi

def control_to_pose(q, goal):
    # Constants
    D = 0.1207
    # Kv = 0.8; Kw = 9; Kg = 6 Best parameters
    Kv = 0.8; Kw = 9; Kg = 6
    r = 0.7

    d_err = np.linalg.norm(goal[:2] - q[:2])
    phi_t = np.arctan2(goal[1] - q[1], goal[0] - q[0])
    alpha = wrap_to_pi(phi_t - goal[2])
    beta = np.arctan(r / d_err)
    if alpha < 0:
        beta = -beta

    if abs(rad2deg(alpha)) > 90:
        vDir = -1
        if abs(np.pi - alpha) < abs(beta):
            phi_err = wrap_to_pi(phi_t - q[2] + alpha)
        else:
            phi_err = wrap_to_pi(phi_t - np.pi - q[2] - beta)
    else:
        vDir = 1
        if abs(alpha) < abs(beta):
            phi_err = wrap_to_pi(phi_t - q[2] + alpha)
        else:
            phi_err = wrap_to_pi(phi_t - q[2] + beta)

    omega = Kw * phi_err
    v = vDir * Kv * d_err

    gamma_reff = np.arctan2(omega * D, v)
    gamma_err = wrap_to_pi(gamma_reff - q[3])

    vs = min(np.sqrt((omega * D)**2 + v**2), 0.2)
    omegas = min(Kg * gamma_err, 13.95)

    stop = (abs(d_err) < 0.01) and (abs(q[2] - goal[2]) < deg2rad(1))

    return vs, omegas, stop


# Handle odometry
def handleOdometry(msg):
    x, y, phi = ams.msgToPose(msg.pose.pose)
    gamma = msg.pose.pose.position.z
    q = [x, y, phi, gamma]

    x_goal = 1
    y_goal = -0.25
    phi_goal = np.pi
    goal = np.array([x_goal, y_goal, phi_goal])

    # Using control_to_pose function
    vs, omegas, stop = control_to_pose(q, goal)

    if stop:
        vs = ws = 0
    

    # Velocity commands message
    msgCmdVel = Twist()
    msgCmdVel.linear.x = vs
    msgCmdVel.angular.z = omegas
    # Publish velocity commands
    pubCmdVel.publish(msgCmdVel)

try:
    rospy.init_node('control_line')

    # Velocity commands publisher.
    pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    # Odometry subscriber
    subOdom = rospy.Subscriber('odom', Odometry, handleOdometry)

    rospy.spin()
except KeyboardInterrupt:
    pass