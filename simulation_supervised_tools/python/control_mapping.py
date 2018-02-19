#!/usr/bin/env python
import rospy
import numpy as np
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
import copy
#
# Maps control from pilot_online and from PS3 on right cmd_vel values
# Possible different states: 
#
cmd_pub = None # control publisher
sup_pub = None # supervision publisher
state_pub = None
state = ""
estimated_yaw = 0
tweak_roll = False
estimated_control=None

def tweak_roll_on_cb(msg):
	global tweak_roll
	if not tweak_roll: 
		print 'tweak roll on'
		tweak_roll=True 

def tweak_roll_off_cb(msg):
	global tweak_roll
	if tweak_roll:
		print 'tweak roll off' 
		tweak_roll=False

def overtake_cb(msg):
	global state
	if state!="user":
		print('[cmd_control] state set to user control.')
		state="user"
		state_pub.publish(state)

def ready_cb(msg):
	global state
	if state!="autopilot":
		state="autopilot"
		print('[cmd_control] state set to autopilot.')
		state_pub.publish(state)

def pilot_cb(data):
	global estimated_yaw, estimated_control
	estimated_yaw = data.angular.z
	estimated_control = copy.deepcopy(data)

	print('pilot control: {}'.format(data.angular))
	# pass

def ps3_cb(data):
	# print('ps3 control: {}'.format(data.angular.z))
	if state=="autopilot" and estimated_control:
		# data.angular.z = 0.5*estimated_yaw
		# data.linear.x = 0.05
		data=copy.deepcopy(estimated_control)
	# compensate yaw with roll:
	if rospy.has_param('tweak_roll') and tweak_roll:
		data.angular.x = np.tanh(rospy.get_param('tweak_roll')*data.linear.x*data.angular.z/10.)
		# print 'change for yaw: ',str(data.angular.z),' is in roll: ',data.angular.x
	cmd_pub.publish(data)
	# print data
	# pass

if __name__=="__main__":
  	rospy.init_node('control_mapping', anonymous=True)
  	state_pub = rospy.Publisher('control_state', String, queue_size=10)
	if rospy.has_param('control'):
		cmd_pub = rospy.Publisher(rospy.get_param('control'), Twist, queue_size=10)
	else:
		raise IOError('[cmd control.py] did not find any control topic!')
	# publish supervised velocity
	sup_pub = rospy.Publisher('/supervised_vel', Twist, queue_size=10)
	
	if rospy.has_param('overtake'): 
		overtake_sub = rospy.Subscriber(rospy.get_param('overtake'), Empty, overtake_cb)
	if rospy.has_param('ready'): 
		ready_sub = rospy.Subscriber(rospy.get_param('ready'), Empty, ready_cb)
	
	pilot_sub = rospy.Subscriber('tf_vel', Twist, pilot_cb)
	ps3_sub = rospy.Subscriber(rospy.get_param('ps3_top'), Twist, ps3_cb)
	
	# tweak_roll_on_sub = rospy.Subscriber('/bebop/tweak_roll_on', Empty, tweak_roll_on_cb)
	# tweak_roll_off_sub = rospy.Subscriber('/bebop/tweak_roll_off', Empty, tweak_roll_off_cb)
	
	# spin() simply keeps python from exiting until this node is stopped	
	rospy.spin()

