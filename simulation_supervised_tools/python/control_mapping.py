#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

#--------------------------------------------------------------------------------------------------------------
# Control mapping is a node that defines which control topic is forwarded to the robot
# according to the control mapping dictionary:
#
# inputs               | TAG 
# -----------------------------------------
# console              | CON
# behavior arbitration | BA
# depth heuristic      | DH
# neural network       | NN
# drive back           | DB
#
# outputs: robot command, supervised command
# 
# The FSM provides a configuration message {robot}_{supervision} command of type string
# example: 
#     "NN_BA" corresponds to neural network publishing on /cmd_vel and behavior arbitration on /sup_vel
#     "CON_NULL" corresponds to only the console publishing on /cmd_vel
#
#--------------------------------------------------------------------------------------------------------------

superviser = "NULL"
pilot = "NULL"
conf_sub = None

# define variables for publishing on robots command and supervision topic
cmd_pub = None # robot command publisher
sup_pub = None # supervision publisher

# define variables for subscriber to different controls
con_sub = None # console subscriber
ba_sub = None # behavior arbitration subscriber
dh_sub = None # depth heuristic subscriber
nn_sub = None # neural network subscriber
db_sub = None # drive back subscriber



def con_cb(data):
	"""Callback on the control coming from console."""
	# check if currently the console can define the robots command
	if pilot=="CON":
		cmd_pub.publish(data)
	# check if currently the console can define the supervision command
	if superviser=="CON":
		sup_pub.publish(data)


def ba_cb(data):
	"""Callback on the control coming from behavior_arbitration."""
	# check if currently the behavior_arbitration can define the robots command
	if pilot=="BA":
		cmd_pub.publish(data)
	# check if currently the behavior_arbitration can define the supervision command
	if superviser=="BA":
		sup_pub.publish(data)


def dh_cb(data):
	"""Callback on the control coming from depth_heuristic."""
	# check if currently the depth_heuristic can define the robots command
	if pilot=="DH":
		cmd_pub.publish(data)
	# check if currently the depth_heuristic can define the supervision command
	if superviser=="DH":
		sup_pub.publish(data)


def nn_cb(data):
	"""Callback on the control coming from neural_network."""
	# check if currently the neural_network can define the robots command
	if pilot=="NN":
		cmd_pub.publish(data)
	# check if currently the neural_network can define the supervision command
	if superviser=="NN":
		sup_pub.publish(data)


def db_cb(data):
	"""Callback on the control coming from drive_back."""
	# check if currently the drive_back can define the robots command
	if pilot=="DB":
		cmd_pub.publish(data)
	# check if currently the drive_back can define the supervision command
	if superviser=="DB":
		sup_pub.publish(data)


def fsm_cb(data):
	"""Callback on the control configuration coming from FSM."""
	global superviser, pilot
	# parse the new configuration
	print data.data
	pilot = data.data.split('_')[0]
	superviser = data.data.split('_')[1]


if __name__=="__main__":
	rospy.init_node('control_mapping', anonymous=True)

	# initialize publishers
	sup_pub = rospy.Publisher('/supervised_vel', Twist, queue_size=10)
	cmd_pub = rospy.Publisher(rospy.get_param('control'), Twist, queue_size=10)
	
	# initialize control subscribers
	con_sub = rospy.Subscriber('con_vel', Twist, con_cb)
	ba_sub = rospy.Subscriber('ba_vel', Twist, ba_cb)
	dh_sub = rospy.Subscriber('dh_vel', Twist, dh_cb)
	nn_sub = rospy.Subscriber('nn_vel', Twist, nn_cb)
	db_sub = rospy.Subscriber('db_vel', Twist, db_cb)	

	# initialize fsm subscriber
	fsm_sub = rospy.Subscriber('control_config', String, fsm_cb)

	# spin() simply keeps python from exiting until this node is stopped	
	rospy.spin()

