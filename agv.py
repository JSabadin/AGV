#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
from agvapi import Agv, findLineEdges
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped

# Foreign libraries
import numpy as np


with Agv() as robot:
  # Handle velocity commands
  def handleCmdVel(msg):
    global robot
    robot.setVel(msg.linear.x, msg.angular.z)



  try:
    rospy.init_node('agv')
    ns = rospy.get_namespace().lstrip('/')
    # Name of the odometry frame
    paramOdomFrameId = rospy.get_param('~odom_frame_id', '{}odom'.format(ns))
    # Name of the AGV frame
    paramAgvFrameId = rospy.get_param('~agv_frame_id', '{}agv'.format(ns))

    # Odometry publisher
    pubOdom = rospy.Publisher('odom', Odometry, queue_size=1)
    # Line sensor publisher
    pubLine = rospy.Publisher('line', LineStamped, queue_size=1)
    # Velocity commands subscriber.
    subCmdVel = rospy.Subscriber('cmd_vel', Twist, handleCmdVel)

    # Line-sensor message
    msgLine = LineStamped()

    # Odometry message
    msgOdom = Odometry()
    msgOdom.header.frame_id = paramOdomFrameId
    msgOdom.child_frame_id = paramAgvFrameId


    def get_difference(current, previous, max_value):
      """ Calculate difference between current and previous considering potential rollover. """
      diff = current - previous

      # Check for rollover!!!
      if diff > max_value / 2:
          diff -= max_value
      elif diff < -max_value / 2:
          diff += max_value

      return diff

    def wrap2pi(angle):
      return np.arctan2(np.sin(angle), np.cos(angle))


    # Odometry initial state
    x, y, phi, gamma = 0.0, 0.0, 0.0, 0.0 # Robot configuration
    fd = 0.0 # Travelled distance of the front cart

    rate = rospy.Rate(50)
    # Initial readings from encoders
    prev_EncodLeft , prev_EncodRight, prev_EncodHeading = robot.getEncoders()
    prev_EncodLeft *= -1

    # Constants 
    # TODO
    # GET NUM_STEPS in DELTA_S and get EncodHeading_offset!
    delta_t = 1/50  # Time step
    DELTA_S = 1   # Distance for calibration
    NUM_STEPS = 111000 # Number of encoder steps for the given DELTA_S distance.
    distance_per_step = DELTA_S / NUM_STEPS
    EncodHeading_offset = 945  # Insert the encoder offset value here 370. We change it 14.11 to 365 because it was off, changed to 945 because new wheels
    MAX_ENCODER_VALUE = 2**13  # Assuming 13-bit encoder

    # Dimensions of robot in meters
    D = 0.1207
    L = 0.043

    while not rospy.is_shutdown():
      t = rospy.Time.now()
      # Read sensors
      robot.readSensors()
      #
      #  Odometry
      #
      # Constants
      # Sample the encoders
      EncodLeft, EncodRight, EncodHeading_raw = robot.getEncoders()  # read encoders
      print(EncodHeading_raw)
      EncodLeft *= -1 # Since it has negative logic

      # Adjust for the offset
      EncodHeading = EncodHeading_raw - EncodHeading_offset

      # Calculate the wheel movement difference considering rollover
      dLeft_raw = get_difference(EncodLeft, prev_EncodLeft, MAX_ENCODER_VALUE)
      dRight_raw = get_difference(EncodRight, prev_EncodRight, MAX_ENCODER_VALUE)

      # Convert the raw differences to movements in meters
      dLeft = dLeft_raw * distance_per_step
      dRight = dRight_raw * distance_per_step

      # Speed of left and right wheel
      vl = dLeft / delta_t
      vr = dRight / delta_t

      # Assume both readings from left and right encoder are the same as they are treated as a single wheel
      vs = (vr + vl) / 2 
      omega_s = (vr - vl)/L

      # Calculate the angular velocity
      gamma = -(EncodHeading / (2**13)) * 2 * np.pi  # Convert encoder reading to steering angle in radians
      phi_dot = vs / D * np.sin(gamma)

      # Kinematics of the tricycle
      x_dot = vs * np.cos(gamma) * np.cos(phi)
      y_dot = vs * np.cos(gamma) * np.sin(phi)
      gamma_dot = omega_s - phi_dot

      # Euler integration
      x += (x_dot * delta_t)
      y += (y_dot * delta_t)
      phi += (phi_dot * delta_t)
      #gamma += (gamma_dot*delta_t)  # We measure gamma...

      # Update the traveled distance
      fd += vs * delta_t

      # Print position and orientation
      
      #print('Encoders: left={}, right={}, heading={}'.format(EncodLeft, EncodRight, prev_EncodHeading))
      
      # Store current readings for the next loop iteration
      prev_EncodLeft = EncodLeft
      prev_EncodRight = EncodRight
      prev_EncodHeading = EncodHeading_raw


      # Debug
      print(f"Pozicija: ({x:.2f}, {y:.2f}), gamma: {wrap2pi(gamma):.2f} rad,  phi: {wrap2pi(phi):.2f} rad, fd : {fd:.2f}")
     



      # End of odometry
      #########################################################################
   

      # Odometry message
      msgOdom.header.stamp = t
      msgOdom.pose.pose = ams.poseToPoseMsg(x, y, wrap2pi(phi))
      msgOdom.pose.pose.position.z = wrap2pi(gamma)
      msgOdom.twist.twist.linear.x = vs * delta_t
      # Publish odometry message
      pubOdom.publish(msgOdom)

      #
      # Line sensor
      #

      # Line-sensor values
      lineValues = robot.getLineValues()
      # Left and right line edge
      edgeLeft, edgeRight = findLineEdges(lineValues)

      # Line-sensor message
      msgLine.header.stamp = t
      msgLine.line.values = lineValues
      msgLine.line.left = edgeLeft if edgeLeft is not None else float('nan')
      msgLine.line.right = edgeRight if edgeRight is not None else float('nan')
      msgLine.line.heading = gamma
      msgLine.line.distance = fd
      # Publish line-sensor message
      pubLine.publish(msgLine)

      rate.sleep()
  except KeyboardInterrupt:
    pass
