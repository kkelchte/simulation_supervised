#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import time
import sys, select, tty, os, os.path
import numpy as np
import commands
from subprocess import call

# Check groundtruth for height
# Log position
# Check depth images for bump 
# Check time for success
# write log when finished and shutdown

# Instantiate CvBridge
bridge = CvBridge()

flight_duration = -1 #amount of seconds drone should be flying, in case of no checking: use -1
delay_evaluation = 3
success=False
shuttingdown=False
ready=False
min_allowed_distance=0.5 #0.8
start_time = 0
log_file ='/tmp/log'
named_log_file =log_file+'_named'
position_log_file = log_file+'_positions.txt'
pidfile="/tmp/.pid"
finished_pub=None
current_pos=[0,0,0]
starting_height = -1
eva_dis=-1
world_name='unk'
positions = []

def shutdown():
  global log_file, named_log_file, position_log_file
  finished_pub.publish(Empty())
  # import pdb; pdb.set_trace() #print("publish finised")
  try: 
    f=open(log_file, 'a')
    message = '{0} \n'.format('success' if success else 'bump')
    f.write(message)
    f.close()
  except :
    print('FAILED TO WRITE LOGFILE: log_file')
  try: 
    f=open(named_log_file, 'a')
    message = '{0} {1} \n'.format('success' if success else 'bump', world_name)
    f.write(message)
    f.close()
  except :
    print('FAILED TO WRITE LOGFILE: named_log_file')
  else:
    time.sleep(1)
  try: 
    f=open(position_log_file, 'a')
    for pos in positions:
      f.write('{0} {1} {2}\n'.format(pos[0],pos[1],pos[2]))
    f.close()
  except :
    print('FAILED TO WRITE LOGFILE: position_log_file')
  else:
    time.sleep(1)
  
  #kill process
  if os.path.isfile(pidfile):
    with open(pidfile, 'r') as pf:
      pid=pf.read()[:-1]
    # gzpid = commands.getstatusoutput('ps -ef | grep gzserver | tail -1')[1].split(' ')[]
    # print("gzserver pid: {}".format(gzpid))
    # call("$(kill -9 "+gzpid+")", shell=True)
    # time.sleep(5)
    time.sleep(1)
    call("$(kill -9 "+pid+")", shell=True)
    

def time_check():
  global start_time, shuttingdown, success
  if start_time == 0:
    start_time = int(rospy.get_time())
  if (int(rospy.get_time()-start_time)) > (flight_duration+delay_evaluation) and not shuttingdown:
    print('time > eva_time----------success!')
    success=True
    shuttingdown=True
    shutdown()
    
def depth_callback(msg):
  global shuttingdown, success
  if shuttingdown or not ready or (rospy.get_time()-start_time)<delay_evaluation: return
  if flight_duration != -1: 
    time_check()
  try:
    min_distance = np.nanmin(bridge.imgmsg_to_cv2(msg))
  except CvBridgeError, e:
    print(e)
  else:
    # print('min distance: ', min_distance)
    if min_distance < min_allowed_distance and not shuttingdown:
      print('bump')
      success=False
      shuttingdown=True
      shutdown()

def gt_callback(data):
  global current_pos, ready, success, shuttingdown, positions
  current_pos=[data.pose.pose.position.x,
                  data.pose.pose.position.y,
                  data.pose.pose.position.z]
  positions.append(current_pos)
  if current_pos[2] > starting_height and not ready and not starting_height==-1:
    #print('EVA: ready!')
    ready_pub.publish(Empty())
    ready = True
  # print 'dis: ',(current_pos[0]**2+current_pos[1]**2)
  # if (current_pos[0] > 52 or current_pos[1] > 30) and not shuttingdown:  
  if eva_dis!=-1 and (current_pos[0]**2+current_pos[1]**2) > eva_dis and not shuttingdown:
    print 'dis > evadis-----------success!'
    success = True
    shuttingdown = True
    shutdown()


if __name__=="__main__":
  rospy.init_node('evaluate', anonymous=True)
  ## create necessary directories
  if rospy.has_param('delay_evaluation'):
    delay_evaluation=rospy.get_param('delay_evaluation')
  if rospy.has_param('flight_duration'):
    flight_duration=rospy.get_param('flight_duration')
  if rospy.has_param('min_allowed_distance'):
    min_allowed_distance=rospy.get_param('min_allowed_distance')
  if rospy.has_param('starting_height'):
    starting_height=rospy.get_param('starting_height')
    if starting_height==-1: #no starting height, so user is flying, so evaluate node should stay ready
      starting_height=0.5
  if rospy.has_param('eva_dis'):
    eva_dis=rospy.get_param('eva_dis')
  if rospy.has_param('log_folder'):
    log_folder=rospy.get_param('log_folder')
  else:
    log_folder = '/tmp/log'
  if rospy.has_param('pidfile'):
    pidfile=rospy.get_param('pidfile')
    pidfile=log_folder+'/'+pidfile
  log_file=log_folder+'/log'
  named_log_file =log_file+'_named'
  position_log_file = log_file+'_positions.txt'
  if rospy.has_param('world_name') :
    world_name = os.path.basename(rospy.get_param('world_name').split('.')[0])
    if 'sandbox' in world_name: world_name='sandbox'
    
  # print '-----------------------------EVALUATION: flight_duration= ',flight_duration

  if rospy.has_param('depth_image'): 
    rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_callback)
  else:
    raise IOError('[evaluate.py] did not find any depth image topic!')
    
  #rospy.Subscriber('/kinect/depth/image_raw', Image, depth_callback)
  #rospy.Subscriber('/ardrone/imu', Imu, imu_callback)
  #rospy.Subscriber('/ready', Empty, ready_callback)
  
  if rospy.has_param('ready'): 
    ready_pub = rospy.Publisher(rospy.get_param('ready'), Empty, queue_size=10)
  if rospy.has_param('finished'): 
    finished_pub = rospy.Publisher(rospy.get_param('finished'), Empty, queue_size=10)
  rospy.Subscriber('/ground_truth/state', Odometry, gt_callback)
  
  # spin() simply keeps python from exiting until this node is stopped	
  rospy.spin()
