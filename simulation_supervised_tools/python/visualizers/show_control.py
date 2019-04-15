#!/usr/bin/env python
import rospy
import time
import sys, select, tty, os, os.path
import numpy as np

# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from sensor_msgs.msg import CompressedImage
import skimage.transform as sm

from std_msgs.msg import Empty
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from std_msgs.msg import String

# plotting
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Instantiate CvBridge
bridge = CvBridge()

#--------------------------------------------------------------------------------------------------------------
#
# Show control is a Graphical User Interface that shows current view of robot combined with different controls
# coming from BA / DH / DB / NN / ..
#
#--------------------------------------------------------------------------------------------------------------

# bebop specific fields
# battery = ''

# control fields
con_ctr = 0
ba_ctr = 0
nn_ctr = 0
dh_ctr = 0
db_ctr = 0

cmd_ctr = None
sup_ctr = None

pilot = ''
superviser = ''
state = ''

# plotting fields
fig=plt.figure(figsize=(10,15))
plt.title('Control Display')
font = cv2.FONT_HERSHEY_PLAIN
current_view = np.zeros((100,150))

implot=plt.imshow(current_view,animated=True)
ctr=1
ctr_arrow=plt.arrow(current_view.shape[0]/2,current_view.shape[1]/2,current_view.shape[0]/2+10*ctr,current_view.shape[1]/2, animated=True)

def build_image():
  """Make interface build on current view of robot.
  Add information from different control fields and current state."""
  view=current_view
  xs=int(view.shape[1]/2)
  ys=int(view.shape[0]/2)  
  
  # plt.arrow()

  # # draw center bar
  # cv2.line(view, (xs, ys-5), (xs,ys+20),(0,0,0), 3)
  # if cmd_ctr: # draw applied control  
  #   cv2.line(view, (xs, ys), (int(xs-150*cmd_ctr.angular.z),ys),(250,50,50), 9)
  #   # draw speed
  #   cv2.line(view, (20, view.shape[0]-35), (20,int(view.shape[0]-35-150*cmd_ctr.linear.x)),(250,50,50), 9)
  #   cv2.putText(view,"linear X",(5,view.shape[0]-10), font, 1,(250,50,50),1)
  
  # if sup_ctr: # draw supervised control  
  #   cv2.line(view, (xs, ys+15), (int(xs-150*sup_ctr.angular.z),ys+15),(0,250,0), 9)
  
  return current_view

def animate(*args):
  implot.set_array(build_image())
  return implot,

def print_dur(start_time, end_time):
  duration = (end_time-start_time)
  m, s = divmod(duration, 60)
  h, m = divmod(m, 60)
  return "%02dm:%02ds" % (m, s)

# def image_callback(msg):
#   global end_time, count
#   try:
#     # Convert your ROS Image message to OpenCV2
#     img = bridge.imgmsg_to_cv2(msg,'bgr8')
#   except CvBridgeError, e:
#     print(e)
#   else:
#     # Draw control
#     xs=int(img.shape[1]/2)
#     ys=int(img.shape[0]/2)
#     #print('size: ',xs,ys)
#     if user_ctr != 0:
#       cv2.line(img, (xs, ys), (int(xs-200*user_ctr),ys),(240,200,0), 9)
#       cv2.line(img, (xs, ys-5), (xs,ys+5),(0,0,0), 3)
#       cv2.putText(img,"supervised_vel",(img.shape[1]-250,img.shape[0]-50), font, 1,(240,200,0),2)  

#     if pilot_ctr != 0:
#       cv2.rectangle(img, (xs, ys-20), (int(xs-200*pilot_ctr),ys+20), (0,0,255),20) 
#       # cv2.line(img, (xs, ys+10), (int(xs-200*pilot_ctr),ys+10),(255,0,0), 20)
#       cv2.rectangle(img, (xs-5, ys-20), (xs+5,ys+20),(0,0,0), 10)
#       cv2.putText(img,"cmd_vel",(img.shape[1]-180,img.shape[0]-15), font, 1,(0,0,255),2)  

#     if battery!='':
#       cv2.putText(img,battery+'%',(img.shape[1]-75,40), font, 1,(240,200,200),2)

#     # Draw state
#     if state != '':
#       cv2.putText(img,state,(10,40), font, 1,(240,200,200),2)
    
#     # if neural_control: 
#     #   end_time = rospy.get_time()
#     if start_time and end_time:
#       cv2.putText(img,print_dur(start_time, end_time),(10,40), font, 1,(240,200,200),2)
         
#     # if neural_control:
#     #   cv2.putText(img,"Control on",(xs+139,40), font, 1,(0,255,0),2)
#     # else:
#     #   cv2.putText(img,"Control off",(xs+137,40), font, 1,(0,0,255),2)
    
#     # cv2.putText(img,"User",(img.shape[1]-105,img.shape[0]-55), font, 1,(240,200,0),2)
    
#     # if recording:
#     #   cv2.circle(img,(30,40), 20, (0,0,255), -1)
    
#     cv2.imshow('Control',img)
#     cv2.waitKey(2)
#     # if recording :
#     #   cv2.imwrite(saving_location+'/'+'{0:010d}.jpg'.format(count),img)
#     #   count+=1

def image_cb(data):
  """Callback on image, saved in current_view field"""
  global current_view
  try:
    # Convert your ROS Image message to OpenCV2
    # img = bridge.imgmsg_to_cv2(data,'bgr8')
    img = bridge.imgmsg_to_cv2(data,'rgb8')
  except CvBridgeError, e:
    print(e)
  else:
    # current_view = img
    current_view = img[::2,::2,:]
    
def compressed_image_cb(data):
  """Callback on image, saved in current_view field"""
  global current_view
  # print('received image')
  try:
    img = bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
  except CvBridgeError, e:
    print(e)
  else:
    # 308x410 to 128x128
    current_view = img[::2,::3,:]
    # current_view = img[::2,::2,:]

def con_cb(data):
  """Callback on the control coming from console."""
  global con_ctr
  con_ctr=data.angular.z

def ba_cb(data):
  """Callback on the control coming from behavior_arbitration."""
  global ba_ctr
  ba_ctr=data.angular.z

def dh_cb(data):
  """Callback on the control coming from depth_heuristic."""
  global dh_ctr
  dh_ctr=data.angular.z

def nn_cb(data):
  """Callback on the control coming from neural_network."""
  global nn_ctr
  nn_ctr=data.angular.z

def db_cb(data):
  """Callback on the control coming from drive_back."""
  global db_ctr
  db_ctr=data.angular.z

def cmd_cb(data):
  """Callback on the control published to robot."""
  global cmd_ctr
  cmd_ctr=data

def sup_cb(data):
  """Callback on the control published to robot."""
  global sup_ctr
  sup_ctr=data


def fsm_cb(data):
  """Callback on the control configuration coming from FSM."""
  global superviser, pilot
  # parse the new configuration
  pilot = data.data.split('_')[0]
  superviser = data.data.split('_')[1]

def state_cb(data):
  """Callback on the control configuration coming from FSM."""
  global state
  state = data.data

# def battery_cb(data):
#   global battery
#   battery=str(data.percent)  

def cleanup():
  """Get rid of the animation on shutdown"""
  plt.close(fig)
  plt.close()

if __name__=="__main__":  

  rospy.init_node('show_control', anonymous=True)
  
  # subscribe to image topic, if not found exit.
  if rospy.has_param('rgb_image'):
    image_topic=rospy.get_param('rgb_image')
    if 'compressed' in image_topic:
      rospy.Subscriber(image_topic, CompressedImage, compressed_image_cb)
    else:
      rospy.Subscriber(image_topic, Image, image_cb)
  else:
    raise IOError('SHOWCONTROL: help I have no image input!')
  
  # initialize control subscribers
  rospy.Subscriber('con_vel', Twist, con_cb)
  rospy.Subscriber('ba_vel', Twist, ba_cb)
  rospy.Subscriber('dh_vel', Twist, dh_cb)
  rospy.Subscriber('nn_vel', Twist, nn_cb)
  rospy.Subscriber('db_vel', Twist, db_cb) 

  rospy.Subscriber(rospy.get_param('control'), Twist, cmd_cb)
  rospy.Subscriber('/supervised_vel', Twist, sup_cb)

  # listen to fsm to define which control is supervising and which is steering
  rospy.Subscriber('control_config', String, fsm_cb)
  rospy.Subscriber('fsm_state', String, state_cb)

  state_pub = None

  # rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, battery_cb)

  if rospy.has_param('graphics'):
    if rospy.get_param('graphics'):
      print("[show_scan_prediction]: showing graphics.")
      anim=animation.FuncAnimation(fig,animate)
      plt.show()
  rospy.on_shutdown(cleanup)

  rospy.spin()
