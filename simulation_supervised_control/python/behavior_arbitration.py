#!/usr/bin/env python
import rospy
import time
import sys, select, tty, os, os.path
import numpy as np

# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt
import matplotlib.animation as animation


#--------------------------------------------------------------------------------------------------------------
#
# Oracle for flying the bebop drone in simulation based on depth from the kinect (10FPS)
# The oracle has different states and starts when start_ba is published:
# 0. do nothing
# 1. take_off and adjust height till drone is at correct height
# 2. wait a little bit to send /go signal so if NN is taking over it received the /go signal
# 3. do obstacle avoidance (with zero turning speed and discrete actions)
#
#--------------------------------------------------------------------------------------------------------------

# Instantiate CvBridge
bridge = CvBridge()

# FSM params
current_state = 0
counter = 0
init_wait = 20 # wait for some time before taking off 
go_wait = 20 # wait for some time while sending 'go'

# Control params
starting_height = 0 # get from ros param dependent on environment defines the height at which drone is flying
adjust_height = 1 # used to adjust the height and keep it at starting_height 
adjust_yaw = 0 # define in which direction to fly

# BA params
clip_distance = 3 #5 tweak for doshico
front_width=40 # define the width of free space in percentage for going straight
horizontal_field_of_view=80 # define percentage of width of depth image used to extract collision
vertical_field_of_view=60 # define percentage of width of depth image used to extract collision
scale_yaw=0.4 #1 
turn_speed=0 # define the forward speed when turning
speed=1.3 

# Publisher fields
action_pub = None
take_off_pub = None
go_pub = None

# Fields used for animating the control
fig=plt.figure(figsize=(10,5))
plt.title('Behavior_arbitration')
barcollection=plt.bar(range(3),[clip_distance for k in range(3)],align='center',color='blue')

depths=np.zeros((3))

def animate(n):
  """Used for visualizing the accumulated depth."""
  for i, b in enumerate(barcollection):
    b.set_height(depths[i])

def cleanup():
  """Get rid of the animation on shutdown"""
  plt.close(fig)
  plt.close()

def depth_callback(data):
  """Extract correct turning direction from the depth image and save it in adjust_yaw."""
  global adjust_yaw, depths

  try:
    # Convert your ROS Image message to OpenCV2
    de = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')#gets float of 32FC1 depth image
  except CvBridgeError as e:
    print(e)
  else:
    de = de[::6,::10]
    shp=de.shape
    # assume that when value is not a number it is due to a too large distance (set to 5m)
    # values can be nan for when they are closer than 0.5m but than the evaluate node should
    # kill the run anyway.
    de=np.asarray([ min(e*1.0, clip_distance) if not np.isnan(e) else clip_distance for e in de.flatten()]).reshape(shp)
    
    # clip left and right part of the image according to the relative field of view
    horizontal_clip = int((100 - horizontal_field_of_view)/200.*de.shape[1])
    vertical_clip = int((100 - vertical_field_of_view)/200.*de.shape[1])
    de = de[vertical_clip:-vertical_clip,horizontal_clip:-horizontal_clip]

    # take minimum of left, center and right side
    bound_index = int((50-front_width/2.)/100*de.shape[1])
    depths=[np.amin(de[:,:bound_index]),
            np.amin(de[:,bound_index:-bound_index]),
            np.amin(de[:,-bound_index:])]
    # choose one discrete action [left, straight, right] according to maximum depth 
    if depths[np.argmax(depths)] == depths[1]: # incase straight is as good as the best, go straight
      adjust_yaw = 0
    else:    
      adjust_yaw = -1*(np.argmax(depths)-1) #as a yaw turn of +1 corresponds to turning left and -1 to turning right
  # adjust_yaw = -1

def gt_callback(data):
  """The ground truth pose is used to adjust the height"""
  global adjust_height
  if data.pose.pose.position.z < (starting_height - 0.1):
    adjust_height = +1
  elif data.pose.pose.position.z > (starting_height + 0.1):
    adjust_height = -1
  else:
    adjust_height = 0

def get_control():
  """Return control conform the current state."""
  control = Twist()
  # adjust height in following states
  if current_state in [1, 2, 3]:
    control.linear.z = adjust_height
  # do obstacle avoidance
  if current_state == 3:
    if adjust_yaw == 0:
      control.linear.x = 1.3
    else:
      control.angular.z = adjust_yaw
  return control

def image_callback(data):
  """Use the frame rate of the images to update the states as well as send the correct control."""
  global current_state, counter
  control = get_control()
  if current_state == 0:
    action_pub.publish(control)
    counter += 1
    if counter > init_wait:
      print("[behavior_arbitration]: {}: State set to 1".format(rospy.get_time()))
      counter = 0
      current_state = 1
  elif current_state == 1:
    action_pub.publish(control)
    take_off_pub.publish(Empty())
    if adjust_height <= 0 : #drone is at correct height so switch to state 2
      print("[behavior_arbitration]: {}: State set to 2".format(rospy.get_time()))
      current_state = 2
  elif current_state == 2:
    action_pub.publish(control)
    go_pub.publish(Empty())
    counter += 1
    if counter > go_wait:
      print("[behavior_arbitration]: {}: State set to 3".format(rospy.get_time()))
      counter = 0
      current_state = 3
  elif current_state == 3:
    action_pub.publish(control)

def scan_callback(data):
  """Callback of lidar scan.
  Defines the adjust_yaw of the send control."""
  global adjust_yaw, depths
  # Preprocess depth:
  ranges=[min(r,clip_distance) if r!=0 else np.nan for r in data.ranges]

  # clip left 45degree range from 0:45 reversed with right 45degree range from the last 45:
  ranges=list(reversed(ranges[:horizontal_field_of_view/2]))+list(reversed(ranges[-horizontal_field_of_view/2:]))

  # turn away from the minimum (non-zero) depth reading
  # discretize 3 bins (:-front_width/2:front_width/2:)
  # range that covers going straight.
  depths=[np.nanmin(ranges[0:horizontal_field_of_view/2-front_width/2]),
          np.nanmin(ranges[horizontal_field_of_view/2-front_width/2:horizontal_field_of_view/2+front_width/2]),
          np.nanmin(ranges[horizontal_field_of_view/2+front_width/2:])]
  
  # choose one discrete action [left, straight, right] according to maximum depth 
  if depths[np.argmax(depths)] == depths[1]: # incase straight is as good as the best, go straight
    adjust_yaw = 0
  else:    
    adjust_yaw = -1*(np.argmax(depths)-1) #as a yaw turn of +1 corresponds to turning left and -1 to turning right


if __name__=="__main__":
  rospy.init_node('Behavior_arbitration', anonymous=True)
  
  # subscribe to depth, rgb image and odometry
  if rospy.has_param('scan'):
    print("[behavior_arbitration]: based on lidar scan")
    rospy.Subscriber(rospy.get_param('scan'), LaserScan, scan_callback) 
  elif rospy.has_param('depth_image'): 
    print("[behavior_arbitration]: based on kinect depth")
    rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_callback)
  else:
    raise IOError('[behavior_arbitration.py] did not find any depth image topic!')
  if rospy.has_param('rgb_image'): 
    rospy.Subscriber(rospy.get_param('rgb_image'), Image, image_callback)
  else:
    raise IOError('[behavior_arbitration.py] did not find any rgb image topic!')
  if rospy.has_param('gt_info'):
    rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_callback)
  
  # extract initial height
  if rospy.has_param('starting_height'): 
    starting_height = rospy.get_param('starting_height')
  
  # make action publisher
  action_pub = rospy.Publisher('ba_vel', Twist, queue_size=1)
  take_off_pub = rospy.Publisher(rospy.get_param('takeoff'), Empty, queue_size=1)
  go_pub = rospy.Publisher('/go', Empty, queue_size=1)

  # only display if depth heuristic is in control or supervision sequence
  control_sequence = {}
  if rospy.has_param('control_sequence'):
    control_sequence=rospy.get_param('control_sequence')
  supervision_sequence = {}
  if rospy.has_param('supervision_sequence'):
    supervision_sequence=rospy.get_param('supervision_sequence')
  
  if rospy.has_param('graphics') and ('BA' in control_sequence.values() or 'BA' in supervision_sequence.values()):
    if rospy.get_param('graphics'):
      print("[depth_heuristic]: showing graphics.")
      anim=animation.FuncAnimation(fig,animate)
      plt.show()
  rospy.on_shutdown(cleanup)

  rospy.spin()
