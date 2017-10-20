#!/usr/bin/env python
import rospy
import numpy as np
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

#
# Give feedback over control received and which control to use
#
size = (100,200)
img = np.zeros(size)+255
state = ''
font = cv2.FONT_HERSHEY_SIMPLEX

def fresh_image():
	img = np.zeros(size)+255
	cv2.putText(img,state,(10,40), font, 1, (0,0,0), 2)
	return img

def control_state_cb(data):
	global img
	state=data.data
	img = fresh_image()	
	
def overtake_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"overtake",(10,80), font, 1, (0,0,0), 2)
def ready_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"ready",(10,80), font, 1, (0,0,0), 2)
def takeoff_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"takeoff",(10,80), font, 1, (0,0,0), 2)
def land_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"land",(10,80), font, 1, (0,0,0), 2)
def emergency_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"emergency",(10,80), font, 1, (0,0,0), 2)
def flattrim_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"flattrim",(10,80), font, 1, (0,0,0), 2)
def rec_on_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"rec_on",(10,80), font, 1, (0,0,0), 2)
def rec_off_cb(data):
	global img
	img = fresh_image()	
	cv2.putText(img,"rec_off",(10,80), font, 1, (0,0,0), 2)

if __name__=="__main__":
  	rospy.init_node('show_console', anonymous=True)
  	state_pub = rospy.Subscriber('control_state', String, control_state_cb)
	if rospy.has_param('overtake'): 
		overtake_sub = rospy.Subscriber(rospy.get_param('overtake'), Empty, overtake_cb)
	if rospy.has_param('ready'): 
		ready_sub = rospy.Subscriber(rospy.get_param('ready'), Empty, ready_cb)
	
	takeoff_sub = rospy.Subscriber(rospy.get_param('takeoff'), Empty, takeoff_cb)
	land_sub = rospy.Subscriber(rospy.get_param('land'), Empty, land_cb)
	emergency_sub = rospy.Subscriber(rospy.get_param('emergency'), Empty, emergency_cb)
	flattrim_sub = rospy.Subscriber(rospy.get_param('flattrim'), Empty, flattrim_cb)
	rec_on_sub = rospy.Subscriber(rospy.get_param('rec_on'), Empty, rec_on_cb)
	rec_off_sub = rospy.Subscriber(rospy.get_param('rec_off'), Empty, rec_off_cb)
	
	
	r = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():	
	    cv2.imshow('Command',img)
	    cv2.waitKey(2)
	    r.sleep()
