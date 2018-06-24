#!/usr/bin/env python
import rospy
import sys, os, time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
import subprocess,shlex

from cv_bridge import CvBridge, CvBridgeError
import cv2

#--------------------------------------------------------------------------------------------------------------------------------
# FSM is a node that structures one episode of an experiment.
# The structure of the FSM is defined by the rosparam fsm_structure in simsup/config/fsm together with a control sequence.
# The FSM connects in each state the correct control node with the robot by communicating on control_config with the control_mapper.
# For each new type of FSM a new configuration file should be defined and the fsm-parser will probably have to grow over time.
#
# The one-state FSM is simply starting everything simultaneously, keeping the state 'running' and providing connecting control  
# of the running state upuntil the ending condition is met and everything is shutdown.
# example configuration:
# state_sequence: ['running']
# control_sequence: {0: 'NN'}
#
# The two-state FSM starts everything in 'idle' state and waits for a signal /go or a correct position to go to 'running'.
# Initially the control is given to 'idle' afterwhich the control is given to 'running'.
# The control can be given back to 'idle' with an /overtake or everything can be shutdown.
# example configuration:
# state_sequence: ['idle','running']
# control_sequence: {0:'BA', 1: 'NN'}
#
# The three-state FSM starts everything in 'idle' and waits for signal /go to start 'running'.
# If a collision is detected it goes to state 'returning' afterwhich the /go signal brings the state back to 'running'.
# Both from the 'running' as from the 'returning' state, the fsm can be send back to 'idle' with an overtake.
# example configuration:
# state_sequence: ['idle','running','returning']
# control_sequence: {0:'CON', 1: 'DH', 2:'DB'}
#
#--------------------------------------------------------------------------------------------------------------

# configuration variables
state_sequence = None
control_sequence = None
supervision_sequence = None
current_state = None

# In case of creating a dataset
save_images = False
start_createds_pub = None
stop_createds_pub = None

# Communicate with gt_listener
start_gt_listener_pub = None
stop_gt_listener_pub = None

# General publishers
state_pub = None
control_map_pub = None

# NN: neural network tensorflow
start_nn_pub = None
stop_nn_pub = None

# dh: depth heuristic (oracle for turtlebot)
start_dh_pub = None
stop_dh_pub = None

# db: drive back
start_db_pub = None

# Log variables:
success = None
start_time = -1
max_duration = -1
max_distance = -1
delay_evaluation = -1
min_depth = -1
world_name='unk'
positions = []
log_folder='~/tensorflow/log/tmp'
clip_distance = 1
field_of_view = 90
run_number = 0 # in case a 3 or 2 fase fsm is running, this counter keeps track of the number of times /go has brought the FSM to state 1
# value is not used for anything specific except for calling datalocation update
data_location = ''
shuttingdown = False

def init():
  """Initialize state 0 which is IDLE or RUNNING depending on the number of states."""
  """When current time > start time  + delay evaluation, control nodes of state '0' should start as well as control mapping."""
  global current_state
  current_state = state_sequence[0]
  state_pub.publish(current_state)
  control_map_pub.publish(control_sequence['0']+"_"+supervision_sequence['0'])

  if "NN" in [control_sequence['0'], supervision_sequence['0']] and start_nn_pub: start_nn_pub.publish(Empty())
  if "DH" in [control_sequence['0'], supervision_sequence['0']] and start_dh_pub: start_dh_pub.publish(Empty())
  if "DB" in [control_sequence['0'], supervision_sequence['0']] and start_db_pub: start_db_pub.publish(Empty())
    
  print("[fsm.py] current state: {}".format(current_state))
  # in case there is only 1 state and save images
  if len(state_sequence)==1: 
    # start saving images
    if save_images and start_createds_pub: start_createds_pub.publish(Empty())
    # start listening to the ground truth position
    if start_gt_listener_pub: start_gt_listener_pub.publish(Empty())

def update_data_location():
  """During a 3-state or 2-state run, create_ds node should change data_location 
  from /path/to/location/00000_worldname to /path/to/location/00001_worldname and so on
  """
  global data_location
  # parse previous run from name
  prev_num=data_location.split('/')[-1].split('_')[0]
  data_location=data_location.replace(prev_num,"{0:05d}".format(int(prev_num)+1))
  rospy.set_param('data_location',data_location)
    
def go_cb(data):
  """Callback on /go to change from 0 or 2 to 1 state"""
  global current_state, shuttingdown, run_number
  if len(state_sequence) >= 2: #if there is only 1 state, the 'go' signal is not used
    shuttingdown = False 
    run_number+=1
    current_state = state_sequence[1]
    state_pub.publish(current_state)
    control_map_pub.publish(control_sequence['1']+"_"+supervision_sequence['1'])
    if "NN" in [control_sequence['1'], supervision_sequence['1']] and start_nn_pub: start_nn_pub.publish(Empty())
    if "DH" in [control_sequence['1'], supervision_sequence['1']] and start_dh_pub: start_dh_pub.publish(Empty())
    if save_images and run_number > 1 and len(data_location) != 0: update_data_location() # increment data location 
    if save_images and start_createds_pub: start_createds_pub.publish(Empty())
    if start_gt_listener_pub: start_gt_listener_pub.publish(Empty())
  print("[fsm.py] current state: {}".format(current_state))

def overtake_cb(data):
  """Callback on /overtake to change from state 1 or 2 to state 0."""
  if save_images and stop_createds_pub: stop_createds_pub.publish(Empty())
  if stop_gt_listener_pub: stop_gt_listener_pub.publish(Empty())

  if "NN" not in [control_sequence['0'], supervision_sequence['0']] and stop_nn_pub: stop_nn_pub.publish(Empty())
  if "DH" not in [control_sequence['0'], supervision_sequence['0']] and stop_dh_pub: stop_dh_pub.publish(Empty())

  init()
  
  
def shutdown():
  global shuttingdown, current_state
  """Shutdown is called from any evaluation method, bringing the fsm potentially in state 2.
  It first writes log information away and shut the process down unless it is a three-state fsm."""
  shuttingdown = True # used to pause other callback functions

  # Warn other process transition from state 0 or 1 to shutdown or state 2
  finished_pub.publish(Empty()) # DEPRECATED
  
  # Pause the saving of images.
  if save_images and stop_createds_pub: stop_createds_pub.publish(Empty())
  
  # Create a new image with the trajectory
  if stop_gt_listener_pub: stop_gt_listener_pub.publish(Empty())


  if stop_nn_pub: stop_nn_pub.publish(Empty())
  if stop_dh_pub: stop_dh_pub.publish(Empty())

  # Go to state 2
  if start_db_pub and len(state_sequence) > 2:
    start_db_pub.publish(Empty())
    control_map_pub.publish(control_sequence['2']+"_"+supervision_sequence['2']) 
    current_state = state_sequence[2]
    state_pub.publish(current_state)
    print("[fsm.py] current state: {}".format(current_state))



  # Log away
  write(log_folder+'/log', '{0} \n'.format('success' if success else 'bump'))
  write(log_folder+'/log_named', '{0} {1} \n'.format('success' if success else 'bump', world_name))
  msg = ""
  for pos in positions: msg = msg + '{0} {1} {2}\n'.format(pos[0],pos[1],pos[2])
  write(log_folder+'/log_positions',msg)

  # Kill simulator from pidfile in log folder.
  pidfile=log_folder+'/.pid'
  if os.path.isfile(pidfile) and len(state_sequence) <= 2:
    with open(pidfile, 'r') as pf:
      pid=pf.read()
    print("[fsm.py]: killing pid {}".format(pid))
    time.sleep(1)
    subprocess.Popen(shlex.split("kill "+pid)).wait()



def time_check():
  """Keep track of the time. If the duration is longer than max_duration shutdown with succes."""
  global start_time, success
  if (int(rospy.get_time()-start_time)) > (max_duration+delay_evaluation) and not shuttingdown:
    print('[fsm.py]: current time {0} > max_duration {1}----------success!'.format(int(rospy.get_time()-start_time),(max_duration+delay_evaluation)))
    success=True
    shutdown()
      
def depth_cb(msg):
  """Read in depth image from kinect and check the minimum value to detect a bump used by the drone."""
  global success
  if shuttingdown or (rospy.get_time()-start_time < delay_evaluation): return
  try:
    de=bridge.imgmsg_to_cv2(msg)
    # print("depth min: {0}, max: {1}, min allowed: {2}".format(np.nanmin(de[de!=0]),np.nanmax(de), min_allowed_distance))
  except CvBridgeError, e:
    print(e)
  else:
    if min_depth != -1 and np.nanmin(de) < min_depth and not shuttingdown:
      print('[fsm.py]: {0}: bump after {1}s'.format(rospy.get_time(), rospy.get_time()-start_time))
      success=False
      shutdown()
      # in case of the drone < kinect readings there is never the situation that you go to state 2 after a bump

def scan_cb(data):
  """Read in depth scan and check the minimum value to detect a bump used by the turtle."""
  if shuttingdown or (rospy.get_time()-start_time < delay_evaluation): return
  # Preprocess depth:
  ranges=[min(r,clip_distance) if r!=0 else np.nan for r in data.ranges]
  # clip left 45degree range from 0:45 reversed with right 45degree range from the last 45:
  ranges=list(reversed(ranges[:field_of_view/2]))+list(reversed(ranges[-field_of_view/2:]))
  # add some smoothing by averaging over 4 neighboring bins
  ranges = [np.nanmean(ranges[i*4:i*4+4]) for i in range(int(len(ranges)/4))]
  # print ranges
  if min_depth != -1 and min(ranges) < min_depth and not shuttingdown:
    print('[fsm.py]: {0}: bump after {1}s'.format(rospy.get_time(), rospy.get_time()-start_time))
    success=False
    shutdown()

def gt_cb(data):
  """Check the traveled distance over the maximum travelled distance before shutdown and keep track of positions for logging.
  This callback has also the special function to start the clock (start_time) of this node and in the same way initialize the controlmapping for the first time. 
  If the gt_cb is not used, the other callbacks will never start as they can't check there delay evaluation."""
  global current_pos, success, positions, start_time, current_state
  current_pos=[data.pose.pose.position.x,
              data.pose.pose.position.y,
              data.pose.pose.position.z]
  if start_time == -1: start_time=rospy.get_time()
  if rospy.get_time() > start_time+delay_evaluation and current_state == None: init()
  if max_duration != -1: time_check()
  positions.append(current_pos)
  if max_distance != -1 and (current_pos[0]**2+current_pos[1]**2) > max_distance and not shuttingdown:
    print '[fsm]: dis > max distance-----------success!'
    success = True
    shutdown()

def write(filename, message):
  """Write some message to some file with error."""
  try:
    f = open(filename, 'a')
    f.write(message)
    f.close
  except:
    print('[fsm]: FAILED TO WRITE LOGFILE: {}'.format(filename))  

if __name__=="__main__":
  rospy.init_node('fsm', anonymous=True)

  # read in configuration
  if rospy.has_param('state_sequence'):
    state_sequence=rospy.get_param('state_sequence')
  if rospy.has_param('control_sequence'):
    control_sequence=rospy.get_param('control_sequence')
  if rospy.has_param('supervision_sequence'):
    supervision_sequence=rospy.get_param('supervision_sequence')
  if rospy.has_param('save_images'):
    save_images=rospy.get_param('save_images')

  print("[fsm.py]: states: {}".format(state_sequence))
  print("[fsm.py]: controls: {}".format(control_sequence))
  print("[fsm.py]: saving images: {}".format(save_images))

  if not state_sequence or not control_sequence:
    print("[fsm.py]: No FSM configuration found so shutting down...")
    sys.exit()

  # publish current state on '/fsm_state'
  state_pub = rospy.Publisher('fsm_state', String, queue_size=10)
  # publish control configuration on '/control_config'
  control_map_pub = rospy.Publisher('control_config', String, queue_size=10)
  # publish when episode is finished and everything is shutting down (although this value topic is DEPRECATED)
  finished_pub = rospy.Publisher('/finished', Empty, queue_size=10)
    
  # initialize subscribers & publishers according to all potential controllers
  # add topic publishers/subscribers specific for control modules
  if 'CON' in control_sequence.values():
    rospy.Subscriber('/overtake', Empty, overtake_cb)
    rospy.Subscriber('/go', Empty, go_cb)
  elif len(state_sequence) > 1: 
    # subscribe to topic /go in case there is more than 1 state, to change from state 0/2 to state 1
    rospy.Subscriber('/go', Empty, go_cb)
  # NN: switch between running and idle where NN computes gradients in case of learning
  if 'NN' in control_sequence.values() or 'NN' in supervision_sequence.values():
    start_nn_pub = rospy.Publisher('/nn_start', Empty, queue_size=10)
    stop_nn_pub = rospy.Publisher('/nn_stop', Empty, queue_size=10)
  # BA: has its own fsm that counts down, takes off, adjusts height and start OA
  # DH: turn on and off with publisehrs
  if 'DH' in control_sequence.values() or 'DH' in supervision_sequence.values():
    start_dh_pub = rospy.Publisher('/dh_start', Empty, queue_size=10)
    stop_dh_pub = rospy.Publisher('/dh_stop', Empty, queue_size=10)
  # DB: drive back is triggered when a bump is detected in a 3 state FSM, db gives a go when the road is free
  if 'DB' in control_sequence.values():
    start_db_pub = rospy.Publisher('/db_start', Empty, queue_size=10)

  # add publishers for starting and stopping the create_dataset node:
  if save_images:
    start_createds_pub=rospy.Publisher('/createds_start', Empty, queue_size=10)
    stop_createds_pub=rospy.Publisher('/createds_stop', Empty, queue_size=10)

  # add publishers for starting and stopping the ground_truth listener:
  start_gt_listener_pub=rospy.Publisher('/gt_listener_start', Empty, queue_size=10)
  stop_gt_listener_pub=rospy.Publisher('/gt_listener_stop', Empty, queue_size=10)

  
  
  # set initial state --> done after time > delay_evaluation
  # current_state = state_sequence[0]
  # state_pub.publish(current_state)

  # Check out parameters for evaluation
  if rospy.has_param('delay_evaluation'): 
    delay_evaluation=rospy.get_param('delay_evaluation')
  if rospy.has_param('max_duration'): 
    max_duration=rospy.get_param('max_duration')
  if rospy.has_param('min_depth'): 
    min_depth=rospy.get_param('min_depth')
  if rospy.has_param('max_distance'): 
    max_distance=rospy.get_param('max_distance')
  if rospy.has_param('gt_info'): 
    rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_cb)
    
  print("[fsm.py]: min_depth: {}".format(min_depth))
  
  if rospy.has_param('log_folder'): 
    loc=rospy.get_param('log_folder')
    if loc[0]=='/':
      log_folder=loc
    else:
      log_folder=os.environ['HOME']+'/tensorflow/log/'+loc
    if not os.path.exists(log_folder): os.makedirs(log_folder)
  print '[fsm]: log folder: {} '.format(log_folder)

  if rospy.has_param('data_location'): # save the data location to change after each new 'go' call
    data_location=rospy.get_param('data_location')
  

  if rospy.has_param('world_name') :
    world_name = os.path.basename(rospy.get_param('world_name').split('.')[0])
    if 'sandbox' in world_name: world_name='sandbox'
  if rospy.has_param('depth_image'):
    if 'scan' in rospy.get_param('depth_image'):
      rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, scan_cb)
    else:
      rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_cb)

  # spin() simply keeps python from exiting until this node is stopped  
  rospy.spin()

