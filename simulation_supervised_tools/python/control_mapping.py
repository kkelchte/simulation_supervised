#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

from nav_msgs.msg import Odometry


import time
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
# keyboard             | KEY
#
# outputs: robot command, supervised command
# 
# The FSM provides a configuration message {robot}_{supervision} command of type string
# example: 
#     "NN_BA" corresponds to neural network publishing on /cmd_vel and behavior arbitration on /supervised_vel
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
key_sub = None # drive back subscriber

# define variables for keeping the start time and current time
# in order to stop control after maximum time
max_time = 1/5. # max time should be around the 1/(rgb frame rate) (1/10FPS) (set dynamically depending on the control rate)
current_time = 0. # last time step got from gazebo or time
control_time = 0. # time in seconds of moment of last send control
clk_sub = None

aggressiveness=1. # define how direct the yaw turn is put on control
adjust_height=-100 
starting_height=0 

def con_cb(data):
  """Callback on the control coming from console."""
  # check if currently the console can define the robots command
  global control_time, max_time
  data.angular.z*=aggressiveness
  if pilot=="CON":
    if adjust_height != -100: data.linear.z = adjust_height
    cmd_pub.publish(data)
    control_time=current_time
    max_time = 1/20. # works at 20FPS < /joy
  # check if currently the console can define the supervision command
  if superviser=="CON":
    sup_pub.publish(data)

def ba_cb(data):
  """Callback on the control coming from behavior_arbitration."""
  # check if currently the behavior_arbitration can define the robots command
  global control_time, max_time
  data.angular.z*=aggressiveness
  if pilot=="BA":
    if adjust_height != -100: data.linear.z = adjust_height
    cmd_pub.publish(data)
    control_time=current_time
    max_time = 1/10. # 10FPS < /kinect
  # check if currently the behavior_arbitration can define the supervision command
  if superviser=="BA":
    sup_pub.publish(data)

def dh_cb(data):
  """Callback on the control coming from depth_heuristic."""
  # check if currently the depth_heuristic can define the robots command
  global control_time, max_time
  data.angular.z*=aggressiveness
  if pilot=="DH":
    if adjust_height != -100: data.linear.z = adjust_height
    cmd_pub.publish(data)
    control_time=current_time
    max_time = 1 #1/5. # 5FPS < /scan: keep same control longer otherwise heuristic gets stuck.
  # check if currently the depth_heuristic can define the supervision command
  if superviser=="DH":
    sup_pub.publish(data)

def nn_cb(data):
  """Callback on the control coming from neural_network."""
  # check if currently the neural_network can define the robots command
  global control_time, max_time
  # print("[control_mapping] changed angular from {}".format(data.angular.z))
  data.angular.z*=aggressiveness
  # print("[control_mapping] to {}".format(data.angular.z))
  if pilot=="NN":
    if adjust_height != -100: data.linear.z = adjust_height
    cmd_pub.publish(data)
    control_time=current_time
    # max_time = 1/10. # 10FPS < /kinect (previous)
    # max_time = 1/5. # 10FPS < /kinect
    # max_time = 1/10. # 10FPS < /ras pi cam
    max_time = 1 # NO stopping
  # check if currently the neural_network can define the supervision command
  if superviser=="NN":
    sup_pub.publish(data)

def db_cb(data):
  """Callback on the control coming from drive_back."""
  # check if currently the drive_back can define the robots command
  global control_time, max_time
  data.angular.z*=aggressiveness
  if pilot=="DB":
    if adjust_height != -100: data.linear.z = adjust_height
    cmd_pub.publish(data)
    control_time=current_time
    max_time = 1/5. # 5FPS < /scan
  # check if currently the drive_back can define the supervision command
  if superviser=="DB":
    sup_pub.publish(data)

def key_cb(data):
  """Callback on the control coming from drive_back."""
  # check if currently the drive_back can define the robots command
  global control_time, max_time
  data.angular.z*=aggressiveness
  if pilot=="KEY":
    if adjust_height != -100: data.linear.z = adjust_height
    cmd_pub.publish(data)
    control_time=current_time
    max_time = 1/5. # 5FPS < /scan
  # check if currently the drive_back can define the supervision command
  if superviser=="KEY":
    sup_pub.publish(data)

def fsm_cb(data):
  """Callback on the control configuration coming from FSM."""
  global superviser, pilot, current_time, control_time, adjust_height
  # parse the new configuration
  print("[control_mapping.py]: received setting: {}".format(data.data))
  pilot = data.data.split('_')[0]
  superviser = data.data.split('_')[1]
  current_time = 0
  control_time = 0
  adjust_height=-100
  cmd_pub.publish(Twist())

def gt_callback(data):
  """The ground truth pose is used to adjust the height if drone is flying"""
  global adjust_height, starting_height

  starting_height=rospy.get_param('starting_height')  
  if data.pose.pose.position.z < (starting_height - 0.1):
    adjust_height = +1
  elif data.pose.pose.position.z > (starting_height + 0.1):
    adjust_height = -1
  else:
    adjust_height = 0
  # print("adjust height to: ",adjust_height)
  if pilot=='NULL':
    data = Twist()
    data.linear.z = adjust_height
    cmd_pub.publish(data)
    control_time=current_time

def clock_cb(data):
  """Callback on the clock of gazebo in order to enforce max 0.1s of action."""
  global current_time
  current_time=int(data.clock.secs)+int(data.clock.nsecs)*10**(-9)
  
if __name__=="__main__":
  rospy.init_node('control_mapping', anonymous=True)

  if rospy.has_param('starting_height'):
    starting_height=rospy.get_param('starting_height')  

  # adjust aggressiveness of turn with by rosparam aggressiveness
  if rospy.has_param('aggressiveness'):
    aggressiveness=rospy.get_param('aggressiveness')

  # initialize fsm subscriber
  fsm_sub = rospy.Subscriber('control_config', String, fsm_cb)

  # initialize publishers
  sup_pub = rospy.Publisher('/supervised_vel', Twist, queue_size=1)
  cmd_pub = rospy.Publisher(rospy.get_param('control'), Twist, queue_size=1)
  
  if rospy.has_param('gt_info') and rospy.get_param('model_name')=='quadrotor':
    rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_callback)
  # initialize control subscribers
  con_sub = rospy.Subscriber('con_vel', Twist, con_cb)
  ba_sub = rospy.Subscriber('ba_vel', Twist, ba_cb)
  dh_sub = rospy.Subscriber('dh_vel', Twist, dh_cb)
  nn_sub = rospy.Subscriber('nn_vel', Twist, nn_cb)
  db_sub = rospy.Subscriber('db_vel', Twist, db_cb) 
  key_sub = rospy.Subscriber('key_vel', Twist, key_cb) 

  # subscribe to the clock of gazebo if gazebo is running
  # else initialize clock of python time
  # if rospy.has_param('gazebo/time_step'): # could be that gazebo is starting up slowly...
  clk_sub = rospy.Subscriber('clock', Clock, clock_cb)

  # rospy.spin()
  # Ensure node is spinning fast enough, reducing delays between time updates
  rate = rospy.Rate(100)  
  while not rospy.is_shutdown():
    # current_time = rospy.get_time()  
    # check how long last control is already send
    # in case it is longer than maximum time
    if (current_time - control_time) > max_time:
      # set control to 0
      # print("[control_mapping.py]: set speed back to zero")
      cmd_pub.publish(Twist())
      control_time = current_time
    # if not rospy.has_param('gazebo/time_step'):
    # incase gazebo has no clock running use ros time
    # current_time = rospy.get_time()
    rate.sleep()

