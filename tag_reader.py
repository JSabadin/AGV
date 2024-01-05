#!/usr/bin/python3
# -*- coding: utf-8 -*-
''' NFC-tag reader

This program publishes a ROS message with the ID of the tag whenever a new tag is detected with MFRC-522 module that is attached to the Raspberry Pi. 

# Installation

This program requires an enabled SPI interface on the Raspberry Pi and Python libraries for that interface.

```bash
#Use sudo raspi-config to enable SPI interface or:
sudo sed -i '/^[# ]*dtparam=spi=.*/d' /boot/config.txt
echo 'dtparam=spi=on' | sudo tee -a /boot/config.txt

sudo apt-get update
sudo apt-get upgrade

sudo apt-get install python3-dev python3-pip
sudo pip3 install RPi.GPIO

cd
git clone https://github.com/lthiery/SPI-Py.git

cd ~/SPI-Py
sudo python3 setup.py install

#cd
#git clone https://github.com/pimylifeup/MFRC522-python.git
#git clone https://github.com/naleefer/MFRC522-python.git
```

# Subscribing to the tag topic

To subscribe to the tag topic, add the following lines to your program.

Add the following line to the import section:
```python
from amsagv_msgs.msg import TagStamped
```

Add the following line to the `__init__` method of your class:
```python
self._subTag = rospy.Subscriber('tag', TagStamped, self._handleTag)
```

Add the following method to your class:
```python
def _handleTag(self, msg):
  print('Tag data: {}'.format(msg.tag.id))
```
'''
import MFRC522
import rospy
from amsagv_msgs.msg import TagStamped
from time import sleep



class TagReader(object):
  REPEAT = 2

  def __init__(self):
    self._reader = None
    
  def __enter__(self):
    self._reader = MFRC522.MFRC522()
    return self
    
  def __exit__(self, *args, **kwargs):
    self._reader.Close_MFRC522()

  def read(self):
    status = None
    uid = None
    n = TagReader.REPEAT
    while n > 0:
      n -= 1
      (status, type) = self._reader.MFRC522_Request(self._reader.PICC_REQIDL)
      if status != self._reader.MI_OK:
        continue
      uid = None
      (status, uid) = self._reader.MFRC522_Anticoll()
      if status == self._reader.MI_OK:
        break

    return TagReader.uid2num(uid)

  @staticmethod
  def uid2num(uid):
    if uid is None or len(uid) != 5:
      return None
    else:
      n = 0
      for i in range(5):
        n = n * 256 + uid[i]
      return n



if __name__ == '__main__':
  with TagReader() as reader:
    try:
      rospy.init_node('tag_reader')

      # Tag publisher
      pub = rospy.Publisher('tag', TagStamped, queue_size=1)
    
      # Tag message
      msg = TagStamped()
    
      last = None
      while not rospy.is_shutdown():
        tagId = reader.read()
        if tagId != last and tagId is not None:
          msg.header.stamp = rospy.Time.now()
          msg.tag.id = int(tagId)
          pub.publish(msg)
        last = tagId
        sleep(0.03)
    except KeyboardInterrupt:
      pass
