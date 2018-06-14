#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

import time
import sys, select, tty, os, os.path
import numpy as np
from subprocess import call
from std_msgs.msg import String
# Instantiate CvBridge
bridge = CvBridge()

# Graphical User Interface that shows current view of drone and cmd_vels but does not receive any input

start_time = None
end_time = None
saving_location = ''
user_ctr = 0
pilot_ctr = 0
state = ''
font = cv2.FONT_HERSHEY_SIMPLEX
count=0
recording=False
save_images=False
battery=''

def print_dur(start_time, end_time):
  duration = (end_time-start_time)
  m, s = divmod(duration, 60)
  h, m = divmod(m, 60)
  return "%02dm:%02ds" % (m, s)

def image_callback(msg):
  global end_time, count
  try:
    # Convert your ROS Image message to OpenCV2
    img = bridge.imgmsg_to_cv2(msg,'bgr8')
  except CvBridgeError, e:
    print(e)
  else:
    # Draw control
    xs=int(img.shape[1]/2)
    ys=int(img.shape[0]/2)
    #print('size: ',xs,ys)
    if user_ctr != 0:
      cv2.line(img, (xs, ys), (int(xs-200*user_ctr),ys),(240,200,0), 9)
      cv2.line(img, (xs, ys-5), (xs,ys+5),(0,0,0), 3)
      cv2.putText(img,"supervised_vel",(img.shape[1]-250,img.shape[0]-50), font, 1,(240,200,0),2)  

    if pilot_ctr != 0:
      cv2.rectangle(img, (xs, ys-20), (int(xs-200*pilot_ctr),ys+20), (0,0,255),20) 
      # cv2.line(img, (xs, ys+10), (int(xs-200*pilot_ctr),ys+10),(255,0,0), 20)
      cv2.rectangle(img, (xs-5, ys-20), (xs+5,ys+20),(0,0,0), 10)
      cv2.putText(img,"cmd_vel",(img.shape[1]-180,img.shape[0]-15), font, 1,(0,0,255),2)  

    if battery!='':
      cv2.putText(img,battery+'%',(img.shape[1]-75,40), font, 1,(240,200,200),2)

    # Draw state
    if state != '':
      cv2.putText(img,state,(10,40), font, 1,(240,200,200),2)
    
    # if neural_control: 
    #   end_time = rospy.get_time()
    if start_time and end_time:
      cv2.putText(img,print_dur(start_time, end_time),(10,40), font, 1,(240,200,200),2)
         
    # if neural_control:
    #   cv2.putText(img,"Control on",(xs+139,40), font, 1,(0,255,0),2)
    # else:
    #   cv2.putText(img,"Control off",(xs+137,40), font, 1,(0,0,255),2)
    
    # cv2.putText(img,"User",(img.shape[1]-105,img.shape[0]-55), font, 1,(240,200,0),2)
    
    # if recording:
    #   cv2.circle(img,(30,40), 20, (0,0,255), -1)
    
    cv2.imshow('Control',img)
    cv2.waitKey(2)
    # if recording :
    #   cv2.imwrite(saving_location+'/'+'{0:010d}.jpg'.format(count),img)
    #   count+=1
    
def state_cb(data):
  global state
  state = str(data.data)
  print('state: '+state)

def recon_callback(msg):
  global recording
  if not recording: 
    print("[show_control]: recording on")
    recording=True
    
def recoff_callback(data):
  global recording
  if recording: 
    print("[show_control]: recording off")
    recording=False
     
def user_cb(data):
  global user_ctr
  #if not ready: return
  #else:
  user_ctr = data.angular.z
  
def pilot_cb(data):
  global pilot_ctr
  #if not ready: return
  #else:
  pilot_ctr = data.angular.z

def battery_cb(data):
  global battery
  battery=str(data.percent)  

if __name__=="__main__":
  if rospy.has_param('saving_location'):
    loc=rospy.get_param('saving_location')
    if loc[0]=='/':
      saving_location=loc
    else:
      saving_location='$HOME/pilot_data/flights/'+loc
  else:
    flight_moment="{0}-{1}-{2}_{3:02d}{4:02d}".format(*list(time.localtime()))
    saving_location='$HOME/pilot_data/flights/'+flight_moment
    print(saving_location)
  
  save_images=False
  if rospy.has_param('save_images'):
    save_images=rospy.get_param('save_images')
  if False:
    print '----------------save images to: ',saving_location
    recording = True
    if not os.path.isdir(saving_location):
      try:
        os.mkdir(saving_location)
      except:
        pass 
  
  supervision=False
  if rospy.has_param('supervision'):
    supervision=rospy.get_param('supervision')
    # supervision=bool(rospy.get_param('supervision')!='false')
    
  rospy.init_node('show_control', anonymous=True)
  rospy.Subscriber('control_state', String, state_cb)

  if rospy.has_param('rgb_image'):
    rospy.Subscriber(rospy.get_param('rgb_image'), Image, image_callback)
  else:
    raise IOError('SHOWCONTROL: help I have no image input!')
  if save_images:
    rospy.Subscriber(rospy.get_param('rec_on'), Empty, recon_callback) 
    rospy.Subscriber(rospy.get_param('rec_off'), Empty, recoff_callback) 
  

  rospy.Subscriber(rospy.get_param('ps3_top'), Twist, user_cb)
  rospy.Subscriber('supervised_vel', Twist, user_cb)
  rospy.Subscriber('tf_vel', Twist, pilot_cb)
  rospy.Subscriber(rospy.get_param('control'), Twist, pilot_cb)
  
  # rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, battery_cb)



  rospy.spin()
