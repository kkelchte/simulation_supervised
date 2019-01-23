#!/usr/bin/env python
import rospy
import sys, os, time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Used for changing pose of model in Gazebo
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty as Emptyservice
from std_srvs.srv import EmptyRequest # for pausing and unpausing physics engine


import numpy as np
import subprocess,shlex

import xml.etree.ElementTree as ET # used to parse goal-tile from world file

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

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

model_state_gazebo_service=None
pause_physics_client=None 

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

# ba: depth heuristic (oracle for turtlebot)
start_ba_pub = None
stop_ba_pub = None

# db: drive back
start_db_pub = None

# Log variables:
success = None
start_time = -1
max_duration = -1
max_distance = -1
goal={}
delay_evaluation = -1
min_depth = -1
world_name='unk'
travelled_distance=0
current_pos=[0,0] #! NOTE: assumption robots spawns at (0,0)
log_folder='~/tensorflow/log/tmp'
clip_distance = 1
field_of_view = 90
smooth_x = 4
run_number = 0 # in case a 3 or 2 fase fsm is running, this counter keeps track of the number of times /go has brought the FSM to state 1
# value is used for calling datalocation update
# value is used to evaluate from time-to-time
evaluate_every=20
data_location = ''
shuttingdown = False
model_name=''

starting_height=-100

def reset():
  """Add entrance of idle state all field variables are reset
  """
  global start_time, current_pos, success, travelled_distance, starting_height
  start_time=-1
  current_pos=[0,0]
  travelled_distance=0
  success=None
  if rospy.has_param('starting_height'):
    starting_height=rospy.get_param('starting_height')

def init():
  """Initialize state 0 which is IDLE or RUNNING depending on the number of states."""
  """When current time > start time  + delay evaluation, control nodes of state '0' should start as well as control mapping."""
  global current_state
  reset()
  current_state = state_sequence[0]
  state_pub.publish(current_state)
  control_map_pub.publish(control_sequence['0']+"_"+supervision_sequence['0'])

  if "NN" in [control_sequence['0'], supervision_sequence['0']] and start_nn_pub: start_nn_pub.publish(Empty())
  if "BA" in [control_sequence['0'], supervision_sequence['0']] and start_ba_pub: start_ba_pub.publish(Empty())
  if "DH" in [control_sequence['0'], supervision_sequence['0']] and start_dh_pub: start_dh_pub.publish(Empty())
  if "DB" in [control_sequence['0'], supervision_sequence['0']] and start_db_pub: start_db_pub.publish(Empty())
    
  print("[fsm.py]:{0}: current state: {1}".format(time.strftime("%Y-%m-%d_%I:%M:%S"),current_state))
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
  try:
    prev_num=data_location.split('/')[-1].split('_')[0]
    data_location=data_location.replace(prev_num,"{0:05d}".format(int(prev_num)+1))
    rospy.set_param('data_location',data_location)
  except:
    print("[fsm]: failed to create new data location from current location {0}".format(data_location))
def go_cb(data):
  """Callback on /go to change from 0 or 2 to 1 state"""
  global current_state, shuttingdown, run_number, max_duration
  shuttingdown = False 
  run_number+=1
  current_state = state_sequence[1]
  state_pub.publish(current_state)
  control_map_pub.publish(control_sequence['1']+"_"+supervision_sequence['1'])
  if len(state_sequence) > 2:
    if (run_number%evaluate_every)== 0:
      print("[fsm.py]: EVALUATE")
      rospy.set_param('evaluate',True)
    else:
      rospy.set_param('evaluate',False)

  if rospy.has_param('max_duration'): 
    max_duration=rospy.get_param('max_duration')
    print("[fsm] set max duration to {0}".format(max_duration))

  if "NN" in [control_sequence['1'], supervision_sequence['1']] and start_nn_pub: start_nn_pub.publish(Empty())
  if "BA" in [control_sequence['1'], supervision_sequence['1']] and start_ba_pub: start_ba_pub.publish(Empty())
  if "DH" in [control_sequence['1'], supervision_sequence['1']] and start_dh_pub: start_dh_pub.publish(Empty())
  if save_images and run_number > 1 and len(data_location) != 0: update_data_location() # increment data location 
  if save_images and start_createds_pub: start_createds_pub.publish(Empty())
  if start_gt_listener_pub: start_gt_listener_pub.publish(Empty())
  print("[fsm.py]:{0}: current state: {1}".format(time.strftime("%Y-%m-%d_%I:%M:%S"),current_state))

def overtake_cb(data):
  """Callback on /overtake to change from state 1 or 2 to state 0."""
  if save_images and stop_createds_pub: stop_createds_pub.publish(Empty())
  if stop_gt_listener_pub: stop_gt_listener_pub.publish(Empty())

  if "NN" not in [control_sequence['0']] and stop_nn_pub: stop_nn_pub.publish(Empty())
  # if "NN" not in [control_sequence['0'], supervision_sequence['0']] and stop_nn_pub: stop_nn_pub.publish(Empty())
  if "DH" not in [control_sequence['0'], supervision_sequence['0']] and stop_dh_pub: stop_dh_pub.publish(Empty())

  init()
  
def shutdown(message):
  global shuttingdown, current_state, start_time
  """Shutdown is called from any evaluation method, bringing the fsm potentially in state 2.
  It first writes log information away and shut the process down unless it is a three-state fsm."""
  shuttingdown = True # used to pause other callback functions

  # Pause simulator not to cause any strange flying away.
  pause_physics_client(EmptyRequest())

  # End run by writing to log file
  print("[fsm]:{0}: {1} ".format(time.strftime("%Y-%m-%d_%I:%M:%S"), message))
  write(log_folder+'/fsm_log','{0} \n'.format(message))

  # Warn other process transition from state 0 or 1 to shutdown or state 2
  # finished_pub.publish(Empty()) # DEPRECATED
  
  # Pause the saving of images.
  if save_images and stop_createds_pub: stop_createds_pub.publish(Empty())  
  
  # Create a new image with the trajectory
  if stop_gt_listener_pub: stop_gt_listener_pub.publish(Empty())

  if stop_nn_pub: stop_nn_pub.publish(Empty())
  if stop_ba_pub: stop_ba_pub.publish(Empty())
  if stop_dh_pub: stop_dh_pub.publish(Empty())

  # Go to state 2
  if start_db_pub and len(state_sequence) > 2:
    start_db_pub.publish(Empty())
    control_map_pub.publish(control_sequence['2']+"_"+supervision_sequence['2']) 
    current_state = state_sequence[2]
    state_pub.publish(current_state)
    print("[fsm.py]:{0}: current state: {1}".format(time.strftime("%Y-%m-%d_%I:%M:%S"),current_state))
  else:
    init()

  shuttingdown = False


def time_check():
  """Keep track of the time. If the duration is longer than max_duration shutdown with succes."""
  global start_time, success
  # if int(rospy.get_time())%60 == 0:
  #   print("[fsm]: current time {0} from start_time {1} vs max_duration {2} plus delay_evaluation {3}".format(rospy.get_time(),start_time,max_duration,delay_evaluation))
  if start_time != -1 and (int(rospy.get_time()-start_time)) > (max_duration+delay_evaluation) and not shuttingdown:
    print('[fsm.py]:{2}: current time {0} > max_duration {1}----------success!'.format(int(rospy.get_time()-start_time),(max_duration+delay_evaluation),time.strftime("%Y-%m-%d_%I:%M:%S")))
    success=True
    shutdown('success')
      
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
    if min_depth != -1 and np.nanmin(de) < min_depth and not shuttingdown and current_state != 'idle':
      print('[fsm.py]: {0}: bump after {1}s'.format(rospy.get_time(), rospy.get_time()-start_time))
      success=False
      shutdown('BUMP DEPTH')
      # in case of the drone < kinect readings there is never the situation that you go to state 2 after a bump

def scan_cb(data):
  """Read in depth scan and check the minimum value to detect a bump used by the turtle."""
  # print("received scan")
  if shuttingdown or (rospy.get_time()-start_time < delay_evaluation): return
  # Preprocess depth:
  ranges=[min(r,clip_distance) if r!=0 else np.nan for r in data.ranges]
  # clip left 45degree range from 0:45 reversed with right 45degree range from the last 45:
  ranges=list(reversed(ranges[:field_of_view/2]))+list(reversed(ranges[-field_of_view/2:]))
  # add some smoothing by averaging over 4 neighboring bins
  ranges = [np.nanmean(ranges[i*smooth_x:i*smooth_x+smooth_x]) for i in range(int(len(ranges)/smooth_x))]
  # print ranges
  if min_depth != -1 and min(ranges) < min_depth and not shuttingdown and current_state != 'idle':
    print('[fsm.py]: {0}: bump after {1}s'.format(rospy.get_time(), rospy.get_time()-start_time))
    success=False
    shutdown('BUMP DEPTH')

def gt_cb(data):
  """Check the traveled distance over the maximum travelled distance before shutdown and keep track of positions for logging.
  This callback has also the special function to start the clock (start_time) of this node and in the same way initialize the controlmapping for the first time. 
  If the gt_cb is not used, the other callbacks will never start as they can't check there delay evaluation."""
  global current_pos, success, start_time, current_state, travelled_distance
  if shuttingdown: return
  if start_time == -1: 
    print("fsm: set start time to: {0}".format(rospy.get_time()))
    start_time=rospy.get_time()
  
  # if rospy.get_time() > start_time+delay_evaluation and current_state == None: init()
  if current_pos != [0,0]:
    travelled_distance+=np.sqrt((current_pos[0]-data.pose.pose.position.x)**2+(current_pos[1]-data.pose.pose.position.y)**2)
  
  # print('[fsm.py]: {0}: travelled: {1} <--> pos: {2}  =: {3}.'.format(rospy.get_time(), travelled_distance, np.sqrt(data.pose.pose.position.x**2+data.pose.pose.position.y**2), np.abs(np.sqrt(data.pose.pose.position.x**2+data.pose.pose.position.y**2)-travelled_distance)))
  current_pos=[data.pose.pose.position.x,
              data.pose.pose.position.y,
              data.pose.pose.position.z]
  
  if rospy.get_time() > start_time+delay_evaluation and current_state == 'idle' and current_pos[2] >= starting_height-0.1: 
    # print("Go")
    go_cb('')

  if max_distance != -1 and (current_pos[0]**2+current_pos[1]**2) > max_distance**2 and not shuttingdown:
  # if max_distance != -1 and travelled_distance > max_distance and not shuttingdown:
    print('[fsm.py]: {0}: travelled distance ({2}) > max distance ({3})-----------success after {1}s'.format(rospy.get_time(), rospy.get_time()-start_time, np.sqrt((current_pos[0]**2+current_pos[1]**2)), max_distance))
    # print('[fsm.py]: {0}: travelled distance ({2}) > max distance ({3})-----------success after {1}s'.format(rospy.get_time(), rospy.get_time()-start_time, travelled_distance, max_distance))
    success = True
    shutdown('success')

  if goal and goal["goal_min_x"] < data.pose.pose.position.x < goal["goal_max_x"] and goal["goal_min_y"] < data.pose.pose.position.y < goal["goal_max_y"] and not shuttingdown:
    print('[fsm.py]: {0}: ({1:0.3f},{2:0.3f}) reached goal tile ({3}) -----------success after {4}s'.format(rospy.get_time(), data.pose.pose.position.x, data.pose.pose.position.y, goal, rospy.get_time()-start_time))
    success = True
    shutdown('success')

def wrench_cb(data):
  """
  Check if the force from the motors of the drone is still giving a minimum z-force.
  If there is no drag force the engines are shut down due to flip over.
  """
  global success
  if shuttingdown or (rospy.get_time()-start_time < delay_evaluation) or current_state != 'running': return
  if data.wrench.force.z < 1:
    print('[fsm.py]: {0}: {2} drag force detected after {1}s'.format(rospy.get_time(), rospy.get_time()-start_time, data.wrench.force.z))
    success = False
    shutdown('BUMP UPSIDEDOWN')

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

  if rospy.has_param('world_name') :
    world_name = os.path.basename(rospy.get_param('world_name').split('.')[0])
    if 'sandbox' in world_name: world_name='sandbox'
  
  print("[fsm.py]: states: {}".format(state_sequence))
  print("[fsm.py]: controls: {}".format(control_sequence))
  print("[fsm.py]: supervisions: {}".format(supervision_sequence))
  print("[fsm.py]: saving images: {}".format(save_images))

  # model_state_gazebo_service=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
  pause_physics_client=rospy.ServiceProxy('/gazebo/pause_physics',Emptyservice)


  # get goal location if params are specified:
  for p in ["goal_max_x","goal_min_x","goal_max_y","goal_min_y"]:
    if rospy.has_param(p):
      goal[p] = rospy.get_param(p)
  if goal: print("[fsm.py]: goal: {0} < x < {1}; {2} < y < {3}".format(goal["goal_min_x"],
                                                                      goal["goal_max_x"],
                                                                      goal["goal_min_y"],
                                                                      goal["goal_max_y"]))

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
    rospy.Subscriber('/overtake', Empty, overtake_cb)
  # NN: switch between running and idle where NN computes gradients in case of learning
  if 'NN' in control_sequence.values() or 'NN' in supervision_sequence.values():
    start_nn_pub = rospy.Publisher('/nn_start', Empty, queue_size=10)
    stop_nn_pub = rospy.Publisher('/nn_stop', Empty, queue_size=10)
  # BA: has its own fsm that counts down, takes off, adjusts height and start OA
  if 'BA' in control_sequence.values() or 'BA' in supervision_sequence.values():
    start_ba_pub = rospy.Publisher('/ba_start', Empty, queue_size=10)
    stop_ba_pub = rospy.Publisher('/ba_stop', Empty, queue_size=10)
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

  
  
  # set initial state (also done after time > start_time + evaluation_time in case the gt_info is published)
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
  if rospy.has_param('evaluate_every'):
    evaluate_every=rospy.get_param('evaluate_every')
  if rospy.has_param('model_name'):
    model_name=rospy.get_param('model_name')
  if rospy.has_param('starting_height'):
    starting_height=rospy.get_param('starting_height')
  
  # used to detect whether motors are still running as they shutdown on flipover.
  rospy.Subscriber("/command/wrench", WrenchStamped, wrench_cb)

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
  
  if rospy.has_param('depth_image'):
    if 'scan' in rospy.get_param('depth_image'):
      rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, scan_cb)
    else:
      rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_cb)

  time.sleep(1)
  init() 
  
  # if not rospy.has_param('gazebo/time_step'): shutdown('CRASH')

  # spin() simply keeps python from exiting until this node is stopped  
  # rospy.spin()

  rate = rospy.Rate(100)  
  while not rospy.is_shutdown():
    if max_duration != -1: 
      time_check()
    rate.sleep()
