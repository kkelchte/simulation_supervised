#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan

import numpy as np
import copy, time
import tf
import sys, select, tty, os, os.path, re

# Instantiate CvBridge
bridge = CvBridge()

saving_location = '/tmp/unknown'
direction = 'none'
coltype = 'none'
save_images = True
index=0
ranges=[]
ranges_front=[]
last_control=[0,0,0,0,0,0]
ready=False
finished=True
start_time=0
rgb_image = None #np.zeros((360,640,3))
skip_first=0

# saving = False # token to avoid overwriting of rgb_image while saving

def write_info(image_type, index):
  if not ready: return
  control=last_control[:] #copy
  cp_ranges=ranges[:]
  cp_ranges_front=ranges_front[:]
  
  with open(saving_location+'/control_info.txt','a') as controlfile:
    controlfile.write("{0:010d} {1[0]} {1[1]} {1[2]} {1[3]} {1[4]} {1[5]}\n".format(index, control))
  scan_msg="{0:010d}".format(index)
  for s in cp_ranges: scan_msg=scan_msg+" "+str(s)
  with open(saving_location+'/scan_info.txt','a') as scanfile: scanfile.write(scan_msg+"\n")
  scan_msg="{0:010d}".format(index)
  for s in cp_ranges_front: scan_msg=scan_msg+" "+str(s)
  with open(saving_location+'/scan_front_info.txt','a') as scanfile: scanfile.write(scan_msg+"\n")
  
def process_rgb(msg, index):
  if (not ready) or finished: return False
  try:
    # Convert your ROS Image message to OpenCV2
    rgb_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough') if turtle else bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError, e:
    print(e)
  else:
    # Save your OpenCV2 image as a jpeg 
    if index > skip_first: 
      print('[create_dataset.py]: {2}: write RGB image {1} to {0}'.format(saving_location, index, rospy.get_time()))
      cv2.imwrite(saving_location+"/RGB/{:010d}.jpg".format(index), rgb_image)
    return True  


def image_callback(msg):
  global index  
  # print('received image for: {}'.format(saving_location))
  if process_rgb(msg, index):
    if index > skip_first: write_info('RGB', index)
    index+=1

def process_depth(msg, index):
  if (not ready) or finished: return False
  try:
    # Convert your ROS Image message to OpenCV2
    im = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')#gets float of 32FC1 depth image
  except CvBridgeError as e:
    print(e)
  else:
    im=im*1/5.*255
    if index > skip_first: 
      print('[create_dataset.py]: {2}: write Depth image {1} to {0}'.format(saving_location, index, rospy.get_time()))
      cv2.imwrite(saving_location+"/Depth/{:010d}.jpg".format(index), im.astype(np.int))
    return True

def depth_callback(msg):
  global index  
  print('received image for: {}'.format(saving_location))
  if process_depth(msg, index):
    if index > skip_first: write_info('Depth', index)
    index+=1

def scan_callback(data):
  global index, ranges, ranges_front
  # print('received scan: {}'.format(saving_location))

  # Preprocess depth:
  ranges=[0.5 if r > 5.0 or r==0 else r for r in data.ranges]
  # clip left 45degree range from 0:45 reversed with right 45degree range from the last 45:
  ranges_front=list(reversed(ranges[:45]))+list(reversed(ranges[-45:]))

def control_callback(data):
  global last_control
  last_control=[data.linear.x,
      data.linear.y,
      data.linear.z,
      data.angular.x,
      data.angular.y,
      data.angular.z]
  
def ready_callback(msg):
  global ready, start_time, finished, saving_location, index
  if not ready and finished:
    if rospy.has_param('saving_location'):
      loc=rospy.get_param('saving_location')
      if loc[0]=='/':
        saving_location=loc
      else:
        saving_location=os.environ['HOME']+'/pilot_data/'+loc
    if os.path.isdir(saving_location):
      # parse last run and create new saving location by incrementing run
      num=len([d for d in os.listdir(saving_location) if os.path.isdir(saving_location+'/'+d) and d.startswith('0')])
    else:
      num=0
    saving_location="{0}/{1:05d}".format(saving_location,num)
    os.makedirs(saving_location+'/RGB')
    ready=True
    finished = False
    start_time=rospy.get_time()
    index=skip_first
    print('[create_dataset]:{0}: ready: saving @ {1}'.format(start_time, saving_location))

def finished_callback(msg):
  global ready, start_time, finished
  if ready and not finished:
    ready=False
    finished = True
    start_time=0
    print('[create_dataset]:{0}: finished'.format(rospy.get_time()))

if __name__=="__main__":  
  rospy.init_node('create_dataset', anonymous=True)
  
  if rospy.has_param('control'):
    rospy.Subscriber(rospy.get_param('control'), Twist, control_callback)
  
  if rospy.has_param('ready'):
    rospy.Subscriber(rospy.get_param('ready'), Empty, ready_callback)
  if rospy.has_param('finished'):
    rospy.Subscriber(rospy.get_param('finished'), Empty, finished_callback)
  if rospy.has_param('overtake'):
    rospy.Subscriber(rospy.get_param('overtake'), Empty, finished_callback)

  save_images = True
  
  

  if rospy.has_param('depth_image'):
    if rospy.get_param('depth_image')!= '/scan':
      rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_callback)
      rospy.Subscriber(rospy.get_param('rgb_image'), Image, image_callback)
    else:
      turtle=True
      # should listen to turtlebot scan instead...
      rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, scan_callback)
      rospy.Subscriber(rospy.get_param('rgb_image'), CompressedImage, image_callback)
  rospy.spin()


