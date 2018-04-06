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
# This node is a filter on the control that is send to the robot. Initially the control is given to the joystick: connecting /ps3_vel to /cmd_vel
# If the joystick gives a 'go' signal the control is given to /tf_vel which is normally the tensorflow control comming from the neural network
# but this is also the drive_back service or the depth heuristic. With X the user can overtake the control giving the control back to the joystick while rest continues publishing on tf_vel.
#
cmd_pub = None # control publisher
sup_pub = None # supervision publisher
state_pub = None
state = ""
estimated_yaw = 0
tweak_roll = False
estimated_control=None

# def tweak_roll_on_cb(msg):
# 	global tweak_roll
# 	if not tweak_roll: 
# 		print 'tweak roll on'
# 		tweak_roll=True 

# def tweak_roll_off_cb(msg):
# 	global tweak_roll
# 	if tweak_roll:
# 		print 'tweak roll off' 
# 		tweak_roll=False

def overtake_cb(msg):
	global state
	if state!="user":
		print('[cmd_control] state set to user control.')
		state="user"
		state_pub.publish(state)

def go_cb(msg):
	global state
	if state!="autopilot":
		state="autopilot"
		print('[cmd_control] state set to autopilot.')
		state_pub.publish(state)

def pilot_cb(data):
	global estimated_yaw, estimated_control
	estimated_yaw = data.angular.z
	estimated_control = copy.deepcopy(data)
	# print('pilot control: {}'.format(data.angular))
	# pass

def ps3_cb(data):
	# print('ps3 control: {}'.format(data.angular.z))
	if state=="autopilot" and estimated_control:
		# data.angular.z = 0.5*estimated_yaw
		# data.linear.x = 0.05
		data=copy.deepcopy(estimated_control)
	# compensate yaw with roll:
	# if rospy.has_param('tweak_roll') and tweak_roll:
	# 	data.angular.x = np.tanh(rospy.get_param('tweak_roll')*data.linear.x*data.angular.z/10.)
	# print 'change for yaw: ',str(data.angular.z),' is in roll: ',data.angular.x
	cmd_pub.publish(data)
	# print data
	# pass

if __name__=="__main__":
  	rospy.init_node('control_mapping', anonymous=True)
  	state_pub = rospy.Publisher('control_state', String, queue_size=10)
	# publish supervised velocity
	sup_pub = rospy.Publisher('/supervised_vel', Twist, queue_size=10)
	
	if rospy.has_param('overtake'): 
		overtake_sub = rospy.Subscriber(rospy.get_param('overtake'), Empty, overtake_cb)
	if rospy.has_param('go'): 
		go_sub = rospy.Subscriber(rospy.get_param('go'), Empty, go_cb)
	
	if rospy.has_param('control'):
		pilot_sub = rospy.Subscriber(rospy.get_param('control'), Twist, pilot_cb)
	else:
		raise IOError('[cmd control.py] did not find any control topic!')
	ps3_sub = rospy.Subscriber(rospy.get_param('ps3_top'), Twist, ps3_cb)
	cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		
	# spin() simply keeps python from exiting until this node is stopped	
	rospy.spin()

