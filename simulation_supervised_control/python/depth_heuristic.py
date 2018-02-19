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

# Check groundtruth for height
# Log position
# Check depth images for bump 
# Check time for success
# write log when finished and shutdown

# Instantiate CvBridge
bridge = CvBridge()

flight_duration = -1 #amount of seconds drone should be flying, in case of no checking: use -1
delay_evaluation = 3
ready=False
finished=True
control_pub=None

def ready_callback(msg):
  global ready, finished
  """ callback function that makes DNN policy starts the ready flag is set on 1 (for 3s)"""
  if not ready and finished:
    print('Control activated.')
    ready = True
    finished = False
    
def finished_callback(msg):
  global ready, finished
  """ callback function that makes DNN policy starts the ready flag is set on 1 (for 3s)"""
  if ready and not finished:
    print('Control deactivated.')
    ready = False
    finished = True

def depth_callback(data):
  global action_pub
  # clip at 0.5m
  data.ranges=[0.7 if r > 0.7 or r==0 else r for r in data.ranges]
  # subsample data from 360 list to 8 bins:
  bins=[sum(data.ranges[45*d:45*(d+1)]) for d in range(8)]
  # focus only on bin [1,0,7,6]
  max_dis_bin=np.argmax([bins[1],bins[0],bins[7],bins[6]])
  
  print("| {0} | {1} | {2} | {3} |".format(bins[1],bins[0],bins[7],bins[6]))

  # with corresponding dicts
  # yaw_dict={0:1, 1:0.5, 2:-0.5, 3:-1}
  yaw_dict={0:-1, 1:-0.7, 2:0.7, 3:1}
  # speed_dict={0:0.2, 1:0.5, 2:0.5, 3:0.2}
  speed_dict={0:0,1:0,2:0,3:0}
  

  # yaw_dict={0:-0.1, 1:-0.3, 2:-0.7, 3:-1, 4:1, 5:0.7, 6:0.3, 7:0.1}
  # yaw_dict={0:0.1, 1:0.3, 2:0.7, 3:1, 4:-1, 5:-0.7, 6:-0.3, 7:-0.1}
  # speed_dict={0:0.5,1:0.1,2:0.,3:0.,4:0,5:0,6:0.1,7:0.5}
  
  msg = Twist()

  msg.linear.x = speed_dict[max_dis_bin]
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.z = yaw_dict[max_dis_bin]

  print("speed: {0} angle: {1} maxbin: {2}".format(speed_dict[max_dis_bin], yaw_dict[max_dis_bin],max_dis_bin))
  action_pub.publish(msg)


if __name__=="__main__":
  rospy.init_node('control_heuristic', anonymous=True)
  # create necessary directories
  if rospy.has_param('delay_evaluation'):
    delay_evaluation=rospy.get_param('delay_evaluation')
  if rospy.has_param('flight_duration'):
    flight_duration=rospy.get_param('flight_duration')
  if rospy.has_param('min_allowed_distance'):
    min_allowed_distance=rospy.get_param('min_allowed_distance')
  
  if rospy.has_param('depth_image'): 
    rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, depth_callback)
  else:
    raise IOError('[depth_heuristic.py] did not find any depth image topic!')
    

  if rospy.has_param('control'): 
    action_pub = rospy.Publisher('/tf_vel', Twist, queue_size=1)

  if rospy.has_param('ready'): 
    ready_pub = rospy.Subscriber(rospy.get_param('ready'), Empty,ready_callback)
  if rospy.has_param('finished'): 
    finished_pub = rospy.Subscriber(rospy.get_param('finished'), Empty,finished_callback)
  
  rospy.spin()
