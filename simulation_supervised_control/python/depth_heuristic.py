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
# Oracle for driving turtlebot in simulation or the real-world based on the LiDAR lazer range finder
# Starts with start_dh topic and publishes control on dh_vel topic.
# Bar plot is left out by default.
#
#--------------------------------------------------------------------------------------------------------------

clip_distance = 5
front_width=50 #47 #define the width of free space in before drifing forward
field_of_view=100 #90

# Instantiate CvBridge
bridge = CvBridge()

control_pub=None

ready = False # toggle on and off with start_dh and stop_dh
finished = True 


fig=plt.figure(figsize=(10,5))
plt.title('Depth_heuristic')
barcollection=plt.bar(range(3),[clip_distance for k in range(3)],align='center',color='blue')

x=np.zeros((3))

def animate(n):
  for i, b in enumerate(barcollection):
    b.set_height(x[i])

def depth_callback(data):
  global action_pub, x
  if not ready or finished: return
  # Preprocess depth:
  ranges=[min(r,clip_distance) if r!=0 else np.nan for r in data.ranges]

  # clip left 45degree range from 0:45 reversed with right 45degree range from the last 45:
  ranges=list(reversed(ranges[:field_of_view/2]))+list(reversed(ranges[-field_of_view/2:]))

  # turn away from the minimum (non-zero) depth reading
  # discretize 3 bins (:-front_width/2:front_width/2:)
  # range that covers going straight.
  x=[np.nanmin(ranges[0:field_of_view/2-front_width/2]),np.nanmin(ranges[field_of_view/2-front_width/2:field_of_view/2+front_width/2]),np.nanmin(ranges[field_of_view/2+front_width/2:])]
  
  if sum(x) == 3*clip_distance: # In case all space is free, go straight. (set 1 as default as argmax takes 0 as default...)
    index=1
  else:
    index=np.argmax(x) # other wise go

  yaw_dict={0:1, # turn left
            1:0, # drive straight
            2:-1} # turn right

  speed_dict={0:0.1, 1:0.3, 2:0.1}  

  # print("[depth_heuristic]: min x: {0}, {1}, {2}, max index: {3}, turn: {4}, speed: {5}".format(x[0],x[1],x[2], index, yaw_dict[index],speed_dict[index]))
  msg = Twist()

  msg.linear.x = speed_dict[index]
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.z = yaw_dict[index]

  # print("[depth_heuristic]: speed: {0} angle: {1} maxbin: {2}".format(speed_dict[max_dis_bin], yaw_dict[max_dis_bin],max_dis_bin))
  action_pub.publish(msg)

def ready_callback(msg):
  """ callback function that makes DH starts and toggles ready"""
  global ready, finished
  if not ready and finished:
    ready = True
    finished = False

def finished_callback(msg):
  """ callback function that makes DH stop and toggles finished"""
  global ready, finished
  if ready and not finished:
    ready = False
    finished = True
    
def cleanup():
  """Get rid of the animation on shutdown"""
  plt.close(fig)
  plt.close()

if __name__=="__main__":
  rospy.init_node('depth_heuristic', anonymous=True)
  
  if rospy.has_param('depth_image'): 
    rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, depth_callback)
  else:
    raise IOError('[depth_heuristic.py] did not find any depth image topic!')
    
  action_pub = rospy.Publisher('dh_vel', Twist, queue_size=1)

  rospy.Subscriber('/dh_start', Empty, ready_callback)
  rospy.Subscriber('/dh_stop', Empty, finished_callback)

  if rospy.has_param('graphics'):
    if rospy.get_param('graphics'):
      print("[depth_heuristic]: showing graphics.")
      anim=animation.FuncAnimation(fig,animate)
      plt.show()
  rospy.on_shutdown(cleanup)

  rospy.spin()
