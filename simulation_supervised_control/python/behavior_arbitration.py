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

import tf

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
adjust_height = 0 # used to adjust the height and keep it at starting_height 

# BA params
## collision avoidance
straight_threshold=0.3 # define from which yaw, we can apply the straight speed vs the turning speed
clip_distance = 5 # tweak for doshico
front_width=40 # define the width of free space in percentage for going straight
horizontal_field_of_view=100 # define percentage of width of depth image used to extract collision
vertical_field_of_view=60 # define percentage of width of depth image used to extract collision
scale_yaw=0.4 #1 
turn_speed=0.8 # define the forward speed when turning
speed=0.8 
avoidance_weight = 1 # how much weight is put to avoidance behavior 
adjust_yaw_collision_avoidance = 0 # define in which direction to fly

## waypoint following
goto_weight = 0 # how much weight is put to folowing weigh point
adjust_yaw_goto = 0 # define in which direction to fly
waypoints=[] # list all waypoints in tuples
current_waypoint_index=0
goto_yaw_amplifier=2
waypoint_reached=0.5 # distance to next waypoint to say drone has reached waypoint
max_deviation=5*np.pi/180 #10

# Publisher fields
action_pub = None
take_off_pub = None
go_pub = None

ready = False
finished = False

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

def gt_callback(data):
  """The ground truth pose is used to adjust the height and goto behavior"""
  global adjust_height, adjust_yaw_goto, current_waypoint_index
  # adjust height
  if data.pose.pose.position.z < (starting_height - 0.1):
    adjust_height = +1
  elif data.pose.pose.position.z > (starting_height + 0.1):
    adjust_height = -1
  else:
    adjust_height = 0
  
  stime=time.time()
  # adjust orientation towards current_waypoint
  quaternion = (data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
  _, _, yaw_drone = tf.transformations.euler_from_quaternion(quaternion)
  
  dy=(waypoints[current_waypoint_index][1]-data.pose.pose.position.y)
  dx=(waypoints[current_waypoint_index][0]-data.pose.pose.position.x)
  
  # print("[behavior_arbitration]: dx {0} dy {1} distance {2} <? min distance {3}".format(dx, dy, np.sqrt(dx**2+dy**2), waypoint_reached))

  if np.sqrt(dx**2+dy**2) < waypoint_reached:
    # update to next waypoint:
    current_waypoint_index+=1
    current_waypoint_index=current_waypoint_index%len(waypoints)
    print("[behavior_arbitration]: reached waypoint: {0}, next waypoint: {1}.".format(waypoints[current_waypoint_index-1],
                                                                                      waypoints[current_waypoint_index]))
    adjust_yaw_goto=0
    return
  else:
    # adjust for quadrants...
    # print("\n\n\ndx {0}, dy {1}".format(dx, dy))
    yaw_goal=np.arctan(dy/dx)
    # print("yaw_goal {0}".format(yaw_goal))
    if np.sign(dx)==-1 and np.sign(dy) ==+1:
      yaw_goal+=np.pi
      # print("adjusted yaw_goal to 2th quadrant: {0} > 0".format(yaw_goal))
    elif np.sign(dx)==-1 and np.sign(dy) ==-1:
      yaw_goal-=np.pi
      # print("adjusted yaw_goal to 3th quadrant: {0} < 0".format(yaw_goal))
    if np.abs(yaw_goal-yaw_drone) > max_deviation:
      adjust_yaw_goto=np.sign(yaw_goal-yaw_drone)
      # if difference between alpha and beta is bigger than pi:
      # swap direction because the other way is shorter.
      if np.abs(yaw_goal-yaw_drone) > np.pi:
        # print("Change yaw turn {0} to {1} because large difference {2}".format(adjust_yaw_goto,-1*adjust_yaw_goto,np.abs(yaw_goal-yaw_drone)))
        adjust_yaw_goto=-1*adjust_yaw_goto
    else:
      adjust_yaw_goto=0
    # print("[behavior_arbitration]: goto_update time: {0:0.2f}, drone yaw: {1}, goal yaw: {2}, adjust_yaw: {3}.".format(time.time()-stime,
    #                                                                                                               yaw_drone,
    #                                                                                                               yaw_goal,
    #                                                                                                               adjust_yaw_goto))


def get_control():
  """Return control conform the current state."""
  control = Twist()
  adjust_yaw=goto_weight*adjust_yaw_goto+avoidance_weight*adjust_yaw_collision_avoidance
  control.angular.z = adjust_yaw
  if adjust_yaw < straight_threshold:
    control.linear.x = speed
  else:
    control.linear.x = turn_speed

  # print("[behavior_arbitration] adjust_yaw_goto {2} + adjust_yaw_collision_avoidance {3} = control [speed {0}; yaw {1}], current_waypoint: {4}".format(control.linear.x, control.angular.z, adjust_yaw_goto, adjust_yaw_collision_avoidance, waypoints[current_waypoint_index]))
  return control

def ready_callback(data):
  """Start node 
  """
  global ready, finished, current_waypoint_index
  if not ready or finished:
    print('[BA] activated.')
    ready = True
    finished = False
    current_waypoint_index=0


def finished_callback(data):
  """Put node in idle state 
  """
  global ready, finished
  if ready or not finished:
    print('[BA] deactivated.')
    ready = False
    finished = True

def image_callback(data):
  """Use the frame rate of the images to update the states as well as send the correct control."""
  global current_state, counter
  if not ready or finished: return
  control=get_control()
  action_pub.publish(control)

def scan_callback(data):
  """Callback of lidar scan.
  Defines the adjust_yaw of the send control."""
  global adjust_yaw_collision_avoidance, depths
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
    adjust_yaw_collision_avoidance = 0
  else:    
    adjust_yaw_collision_avoidance = -1*(np.argmax(depths)-1) #as a yaw turn of +1 corresponds to turning left and -1 to turning right

def depth_callback(data):
  """Extract correct turning direction from the depth image and save it in adjust_yaw_collision_avoidance."""
  global adjust_yaw_collision_avoidance, depths

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
      adjust_yaw_collision_avoidance = 0
    else:    
      adjust_yaw_collision_avoidance = -1*(np.argmax(depths)-1) #as a yaw turn of +1 corresponds to turning left and -1 to turning right
  # adjust_yaw_collision_avoidance = -1

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
  

  rospy.Subscriber('/ba_start', Empty, ready_callback)
  rospy.Subscriber('/ba_stop', Empty, finished_callback)

  rospy.Subscriber(rospy.get_param('rgb_image'), Image, image_callback)
  if rospy.has_param('rgb_image'): 
      rospy.Subscriber(rospy.get_param('rgb_image'), Image, image_callback)
  else:
    raise IOError('[behavior_arbitration.py] did not find any rgb image topic!')

  # make action publisher
  action_pub = rospy.Publisher('ba_vel', Twist, queue_size=1)
  take_off_pub = rospy.Publisher(rospy.get_param('takeoff'), Empty, queue_size=1)

  # only display if depth heuristic is in control or supervision sequence
  control_sequence = {}
  if rospy.has_param('control_sequence'):
    control_sequence=rospy.get_param('control_sequence')
  supervision_sequence = {}
  if rospy.has_param('supervision_sequence'):
    supervision_sequence=rospy.get_param('supervision_sequence')
  
  if rospy.has_param('graphics') and ('BA' in control_sequence.values() or 'BA' in supervision_sequence.values()):
    if rospy.get_param('graphics') and False:
      print("[depth_heuristic]: showing graphics.")
      anim=animation.FuncAnimation(fig,animate)
      plt.show()
    rospy.on_shutdown(cleanup)

  if rospy.has_param('gt_info'):
    rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_callback)

  # if go_to behavior is used.
  if rospy.has_param('waypoints') and rospy.has_param('gt_info'):
    waypoints=rospy.get_param('waypoints')
    print("[behavior_arbitration]: found following waypoints:{0}.".format(waypoints))
    if rospy.has_param('goto_weight'):
      goto_weight=rospy.get_param('goto_weight')
      avoidance_weight=1-goto_weight
      print("[behavior_arbitration] goto_weight {0} and avoidance_weight {1}".format(goto_weight, avoidance_weight)) 
    else:
      goto_weight=0.5
      avoidance_weight=0.5

    #DEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUG
    #DEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUG
    # goto_weight=1
    # avoidance_weight=0.0
    #DEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUG
    #DEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUGDEBUG


  rospy.spin()
