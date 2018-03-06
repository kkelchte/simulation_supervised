#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError

import cv2

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

import time
import sys, select, tty, os, os.path
import numpy as np
import commands

from subprocess import call

# Check groundtruth for height
# Log position
# Check depth images for bump 
# Check time for success
# write log when finished and shutdown

# Instantiate CvBridge
bridge = CvBridge()

turtle=False # boolean indicating we are working with a trutlebot 


ready=False
finished=True
start_time = 0
log_file ='/tmp/log'

delays=[0,0,0]

last_depth=0
last_rgb=0
last_ctr=0

def ctr_callback(data):
  global last_ctr
  if finished or not ready: return
  now=rospy.get_rostime()
  current_time=now.secs+now.nsecs*10e-10
  print("{2}: {0}s {1} ns".format(now.secs, now.nsecs,current_time))

def rgb_callback(data):
  global last_rgb
  if finished or not ready: return
    
def depth_callback(data):
  global last_depth
  if finished or not ready: return

def scan_callback(data):
  global last_depth
  if finished or not ready: return

  # Preprocess depth:
  ranges=[0.5 if r > 0.5 or r==0 else r for r in data.ranges]

def ready_callback(msg):
  global ready, finished
  """Called when ready is received from joystick (traingle), only used when working with turtlebot"""
  if not ready and finished:
    print('[monitor.py]: ready')
    ready = True
    finished = False

def finished_callback(msg):
  global ready, finished
  """ Called when ready is received from joystick (x) when user is taking over, only used when working with turtlebot"""
  if ready and not finished:
    print('[monitor.py]: finished')
    ready = False
    finished = True

def gt_callback(data):
  if finished or not ready: return
  current_pos=[data.pose.pose.position.x,
                  data.pose.pose.position.y,
                  data.pose.pose.position.z]

if __name__=="__main__":
  rospy.init_node('monitor', anonymous=True)
  start_time=rospy.get_time()

  if rospy.has_param('log_folder'):
    log_folder=rospy.get_param('log_folder')
  else:
    log_folder = '/tmp/log'
  log_file=log_folder+'/log_monitor'
  
  if rospy.has_param('control'):
    rospy.Subscriber(rospy.get_param('control'), Twist, ctr_callback)

  if rospy.has_param('rgb_image'):
    rospy.Subscriber(rospy.get_param('rgb_image'), Image, rgb_callback)
  
  if rospy.has_param('depth_image'):
    if rospy.get_param('depth_image')!= '/scan':
      rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_callback)
    else:
      print("[monitor.py]: turtle robot")
      turtle=True
      # should listen to turtlebot scan instead...
      rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, scan_callback)
  
  if rospy.has_param('ready'): 
    rospy.Subscriber(rospy.get_param('ready'), Empty, ready_callback)

  if rospy.has_param('finished'): 
    rospy.Subscriber(rospy.get_param('finished'), Empty, finished_callback)
  
  if rospy.has_param('gt_info'): 
    rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_callback)
  
  # spin() simply keeps python from exiting until this node is stopped	
  rospy.spin()
