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
#
#--------------------------------------------------------------------------------------------------------------

# Instantiate CvBridge
bridge = CvBridge()

flight_duration = -1 #amount of seconds drone should be flying, in case of no checking: use -1
delay_evaluation = 3
ready=False
finished=True
control_pub=None

fig=plt.figure(figsize=(10,5))
plt.title('Depth_heuristic')

x=np.zeros((3))
barcollection=plt.bar(range(3),[0.5 for k in range(3)],align='center',color='blue')

def animate(n):
  for i, b in enumerate(barcollection):
    b.set_height(x[i])

def ready_callback(msg):
  global ready, finished
  """ callback function that makes DNN policy starts the ready flag is set on 1 (for 3s)"""
  if not ready and finished:
    print('[depth_heuristic]: Control activated.')
    ready = True
    finished = False
    
def depth_callback(data):
  global action_pub, x
  # clip at 0.5m and make 'broken' 0 readings also 0.5
  ranges=[0.5 if r > 0.5 or r==0 else r for r in data.ranges]
  # clip left 45degree range from 0:45 reversed with right 45degree range from the last 45:
  ranges=list(reversed(ranges[:45]))+list(reversed(ranges[-45:]))

  # turn away from the minimum (non-zero) depth reading
  # discretize 3 bins (:-front_width/2:front_width/2:)
  # range that covers going straight.
  front_width=47
  x=[min(ranges[0:45-front_width/2]),min(ranges[45-front_width/2:45+front_width/2]),min(ranges[45+front_width/2:])]
  
  index=np.argmax(x)

  yaw_dict={0:1, # turn left
            1:0, # drive straight
            2:-1} # turn right

  speed_dict={0:0.1, 1:0.3, 2:0.1}  

  print("[depth_heuristic]: min x: {0}, {1}, {2}, max index: {3}, turn: {4}, speed: {5}".format(x[0],x[1],x[2], index, yaw_dict[index],speed_dict[index]))
  

  msg = Twist()

  msg.linear.x = speed_dict[index]
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.z = yaw_dict[index]

  # print("[depth_heuristic]: speed: {0} angle: {1} maxbin: {2}".format(speed_dict[max_dis_bin], yaw_dict[max_dis_bin],max_dis_bin))
  action_pub.publish(msg)


if __name__=="__main__":
  rospy.init_node('depth_heuristic', anonymous=True)
  
  if rospy.has_param('depth_image'): 
    rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, depth_callback)
  else:
    raise IOError('[depth_heuristic.py] did not find any depth image topic!')
    
  action_pub = rospy.Publisher('/dh_vel', Twist, queue_size=1)

  ready_pub = rospy.Subscriber('dh_start', Empty,ready_callback)  
  # finished_pub = rospy.Subscriber('dh_stop', Empty,finished_callback)
  
  anim=animation.FuncAnimation(fig,animate)
  plt.show()

  rospy.spin()
