#!/usr/bin/python3
# -*- coding: utf-8 -*-
''' Interface for serial communication with the AGV driver

It contains a simple test program that sets constant motor velocities and outputs sensor readings.
'''
import serial
import struct
from time import sleep, time
from ams import wrapToPi
from math import pi



class Agv(object):
  WHEEL_VEL_GAIN = 1000.0
  AXIS_LENGTH = 0.075 # Axis length, in m
  WHEEL_MAX_V = 0.3 # Max wheel velocity, in m/s
  REPEAT = 10 # Number of start/stop command repeats
  TIMEOUT = 0.02 # Start/stop command timeout

  def __init__(self, port='/dev/ttyS0', baud=115200):
    self._serial = serial.Serial()
    self._serial.baudrate = baud
    self._serial.port = port
    self._serial.timeout = 0
    self._buff = bytearray()

    self._data = [0]*28

  def __enter__(self):
    self.open()
    return self

  def __exit__(self, *args, **kwargs):
    self.close()

  def open(self):
    self._serial.open()
    
    m, s = False, False
    M, S = True, True
    i = Agv.REPEAT
    while m != M and s != S and i > 0:
      self.setPowerMode(motors=M, sensors=S)
      sleep(Agv.TIMEOUT)
      self.readSensors()
      t, m, s = self.getStatus()
      i -= 1
    if i==0:
      raise SystemError('Unable to setup the AGV controller.')

  def close(self):
    for i in range(Agv.REPEAT):
      self.setWheelVel() # Stop the robot
      sleep(Agv.TIMEOUT)
    for i in range(Agv.REPEAT):
      self.setPowerMode(motors=False, sensors=False) # Enable power-saving mode
      sleep(Agv.TIMEOUT)
    
    self._serial.close()

  #Set tangential and angular velocity, in m/s and rad/s, respectively
  def setVel(self, v=0.0, omega=0.0):
    vl = -v+Agv.AXIS_LENGTH/2.0*omega
    vr = -v-Agv.AXIS_LENGTH/2.0*omega
    self.setWheelVel(vl, vr)

  # Set wheel velocities, in m/s
  def setWheelVel(self, left=0.0, right=0.0):
    cmd = [left, -right]
    chk = 0
    for i in range(len(cmd)):
      cmd[i] = Agv._clamp(cmd[i], -Agv.WHEEL_MAX_V, Agv.WHEEL_MAX_V)
      cmd[i] = int(cmd[i]*Agv.WHEEL_VEL_GAIN)
      cmd[i] = Agv._clampShort(cmd[i])
      chk += (cmd[i]&0xFF) + (cmd[i]>>8)
    chk += 0x5A + 0x20 + 0x04
    msg = struct.pack('<BBBhhB', 0x5A, 0x20, 0x04, cmd[0], cmd[1], (-chk) & 0xFF)
    #print("TX: "+":".join("{:02X}".format(x) for x in msg)) # Debug
    self._serial.write(msg)

  def setPowerMode(self, motors=False, sensors=False):
    chk = 0
    a = int(motors)
    b = int(sensors)
    chk = 0x5A + 0x21 + 0x02 + a + b
    msg = struct.pack('<BBBBBB', 0x5A, 0x21, 0x02, a, b, (-chk) & 0xFF)
    #print("TX: "+":".join("{:02X}".format(x) for x in msg)) # Debug
    self._serial.write(msg)
    sleep(Agv.TIMEOUT)

  def setWatchdog(self, period):
    period = Agv._clampUShort(period)
    chk = 0x5A + 0x22 + 0x01 + (period>>8) + (period&0xFF)
    msg = struct.pack('<BBBBBB', 0x5A, 0x22, 0x01, period, (-chk) & 0xFF)
    #print("TX: "+":".join("{:02X}".format(x) for x in msg)) # Debug
    self._serial.write(msg)
    sleep(Agv.TIMEOUT)

  # Read sensors
  def readSensors(self):
    msg = self._serial.read((3+40+1)*20)
    if len(msg):
      #print("RX: "+":".join("{:02X}".format(x) for x in msg)) # Debug
      self._buff += bytearray(msg)
      n = len(self._buff)
      i = 0
      while n-i >= 4:
        head, cmd, length = struct.unpack('<BBB', self._buff[i:i+3])
        if head == 0x5A:
          if n-i >= length + 1 + 3:
            chk = sum(self._buff[i:i+3+length+1])
            if chk & 0xFF == 0:
              if cmd == 0x10:
                self.data = struct.unpack('<Iiii8B8B8B', self._buff[i+3:i+3+length])
              i += 3 + length + 1
              continue
          else:
            break
        i += 1
      self._buff = self._buff[i:]

  def getEncoders(self):
    return self.data[1], self.data[2], self.data[3]
    
  def getLineValues(self, mode=0):
    mode %= 3
    return self.data[4+8*mode:4+8*mode+7]
    
  def getStatus(self):
    return self.data[0] & 0x00FFFFFF, (self.data[0] & 0x01000000) != 0, (self.data[0] & 0x02000000) != 0
    
  def printSensors(self):
    print('\033c')
    print('Status: t={} ms, motors={}, sensors={}'.format(*self.getStatus()))
    print('Encoders: left={}, right={}, aux={}'.format(*self.data[1:4]))
    print('Line sensor:')
    print('  Individual: {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}'.format(*self.data[4:4+8]))
    print('  All on:     {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}'.format(*self.data[12:12+8]))
    print('  All off:    {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}'.format(*self.data[20:20+8]))
    
    print('  Individual   All on       All off')
    print('  ----------   ----------   ----------')
    tmpl = '  {:10s}   {:10s}   {:10s}'
    for i in range(8):
      a = [int(x*10/255) for x in self.data[4+i:21+i:8]]
      print(tmpl.format(*['o'*x for x in a]))

  @staticmethod
  def _clamp(num, low, high):
    return max(min(high, num), low)
  @staticmethod
  def _clampShort(num):
    return Agv._clamp(num, -32768, 32767)
  @staticmethod
  def _clampUShort(num):
    return Agv._clamp(num, 0, 65536)



def findLineEdges(lineValues):
  smin = min(lineValues)
  smax = max(lineValues)
  av = sum(lineValues) / 7    
  threshold = (smin + smax) / 2

  edgeRight = None
  edgeLeft = None
  
  if smax - smin > 80:
    lineEl = [i for i in range(len(lineValues)) if lineValues[i] > threshold]

    # Right edge
    if lineEl[0] == 0:
      edgeRight = -1
    else:
      vs = lineValues[lineEl[0]-1:lineEl[0]+1]
      if abs(vs[1] - vs[0]) > 0:
        edgeRight = ((lineEl[0] + float((threshold - vs[0])) / (vs[1] - vs[0])) - 4) / 2.5
      else:
        edgeRight = (lineEl[0] + 0.5 - 4) / 2.5
            
    # Left edge
    if lineEl[-1] == 6:
      edgeLeft = 1
    else:
      vs = lineValues[lineEl[-1]:lineEl[-1]+2]
      if abs(vs[1] - vs[0]) > 0:
        edgeLeft = ((lineEl[-1] + float((threshold - vs[1])) / (vs[1] - vs[0])) - 2.5) / 2.5
      else:
        edgeLeft = (lineEl[-1] - 2.5) / 2.5
  elif smax > 150:
    # On the line
    edgeRight = -1.0
    edgeLeft = 1.0
      
  return edgeLeft, edgeRight



if __name__ == '__main__':
  with Agv() as robot:
    try:
      k = 0
      while True:
        robot.readSensors()
        lineValues = robot.getLineValues()
        edgeLeft, edgeRight = findLineEdges(lineValues)
        k += 1
        if k % 20 == 0:
          robot.printSensors()
          print('Line edges: left={}, right={}'.format(edgeLeft, edgeRight))
        robot.setVel(0.0, 1.0)
        sleep(0.01)
    except KeyboardInterrupt:
      pass
