#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import time
import sys, select, tty, os, os.path
import numpy as np
import commands
from subprocess import call

import matplotlib.pyplot as plt
import matplotlib.animation as animation

#--------------------------------------------------------------------------------------------------------------
#
# Drive back turns the turtlebot away from a bump by rotating around its z-axis.
# When the road is free /go is published an empty message to bring fsm in running state.
# The service is started with an empty message on '/db_start'
# Control is published on /db_vel
# Node remains idle as long as drive_back_start is not called.
# By default bar plot animation is left out.
# 
#--------------------------------------------------------------------------------------------------------------


state='idle' #idle or driving back...
num_bins=36 # should be able to divide 360

fig=plt.figure(figsize=(10,5))
plt.title('Drive_back')
anim = None

x=np.zeros((num_bins))
barcollection=plt.bar(range(num_bins),[0.5*360./num_bins for k in range(num_bins)],align='center',color='blue')

def animate(n):
  for i, b in enumerate(barcollection):
    b.set_height(x[i])

def drive_back_callback(msg):
  global state
  """ callback function that makes DNN policy starts the ready flag is set on 1 (for 3s)"""
  if state != 'driving':
    state='driving'
    print('[drive_back]: Driving back service started.')


def depth_callback(data):
  global action_pub, x, state

  if state == 'idle': return
  # 1. Process lazer range data: getting the closest octant

  # clip at 0.5m and make 'broken' 0 readings also 0.5
  ranges=[0.5 if r > 0.5 or r==0 else r for r in data.ranges]

  # discriteze in 360/num_bins octants

  # ranges=[np.sum(ranges[10*i:10*(i+1)]) for i in range(num_bins)]
  ranges=[np.sum(ranges[360/num_bins*i:360/num_bins*(i+1)]) for i in range(num_bins)]
  x=np.array(ranges)
  
  # 2. check if current depth is indicating a free road quit driving and go to idle state
  # print("[drive_back]: min index: {0}".format(np.argmin(ranges)))

  if 3./8*num_bins < np.argmin(ranges) and np.argmin(ranges) < 5./8*num_bins:
    print("[drive_back]: Drive back to free road is done.")
    state='idle'
    drive_back_pub.publish(Empty())
  else:
    # 3. else turn so that road becomes free.
    # print('[drive_back]: turning turning turning...')
    msg = Twist()
    msg.angular.z = -1
    action_pub.publish(msg)

def cleanup():
  """Get rid of the animation on shutdown"""
  plt.close(fig)
  plt.close()

if __name__=="__main__":
  rospy.init_node('drive_back', anonymous=True)
  
  if rospy.has_param('depth_image'): 
    rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, depth_callback)
  else:
    raise IOError('[drive_back.py] did not find any depth image topic!')
    

  action_pub = rospy.Publisher('db_vel', Twist, queue_size=1)

  rospy.Subscriber('/db_start', Empty, drive_back_callback)
  drive_back_pub = rospy.Publisher('/go', Empty, queue_size=1)
  
  if rospy.has_param('graphics'):
    if rospy.get_param('graphics'):
      print("[drive_back]: showing graphics.")
      anim=animation.FuncAnimation(fig,animate)
      plt.show()

  rospy.on_shutdown(cleanup)

  rospy.spin()
