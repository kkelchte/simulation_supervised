#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import numpy as np
import copy, time
import tf
import sys, select, tty, os, os.path, re

from std_srvs.srv import Empty as Emptyservice
from std_srvs.srv import EmptyRequest # for pausing and unpausing physics engine


# Instantiate CvBridge
bridge = CvBridge()


#--------------------------------------------------------------------------------------------------------------------------------
#
# CREATE_DATASET.PY
#
# Create dataset is a node that combines all the required steps for saving information into a dataset.
# The function starts saving on /createds_start and stops on /createds_stop coming from fsm or console.
# The log-information is written away for each image at the data_location from rosparam log_folder.
# The rgb images are saved in log_folder/RGB, while depth images are saved at log_folder/Depth.
# The log information follows the index of the RGB and Depth image.
# There is an index counter shared between depth and RGB that is incremented by both image inputs.
#
# Callback functions are defined for:
#  - RGB (kinect and normal) images
#  - Depth (kinect) images
#  - Lazer scans
#  - GT_odometry / imu info
#  - Control /cmd_vel
#  - Supervision /sup_vel
#
# Possible extensions:
#  - Listen to all control (oracle/nn/console)
#  - Save depth predictions
#  - Log delays
#
#--------------------------------------------------------------------------------------------------------------------------------


data_location = None
index=0
index_dict={'left':0,'right':0} #Extra indices for recovery cameras
last_supervised_control=[0,0,0,0,0,0]
last_control=[0,0,0,0,0,0]
last_scan=[]
last_gt=[0,0,0]
last_position=[]
last_odom=[]
T_pg=[] # transformation of previous pose in global coordinates
ready=False
finished=True
recovery=False

# keep track of how fast images arrive (first thing at callback)
# and how fast they are written away
rgb_cb_rate=[]
rgb_cb_ts=0
rgb_write_rate=[]
rgb_write_ts=0

skip_first=4 # don't save the first images as the sensor is still starting up.

try:
  pause_physics_client=rospy.ServiceProxy('/gazebo/pause_physics',Emptyservice)
  unpause_physics_client=rospy.ServiceProxy('/gazebo/unpause_physics',Emptyservice)
except:
  pass

def process_rgb_compressed(msg, index):
  """If ready-state: go from serial to rgb image and save it."""
  if (not ready) or finished: return False
  try:
    rgb_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
  except CvBridgeError as e:
    print(e)
  else:
    # Save your OpenCV2 image as a jpeg 
    if index > skip_first: 
      # print('[create_dataset.py]: {2}: write RGB image {1} to {0}'.forma, index, rospy.get_time()))
      cv2.imwrite(data_location+"/RGB/{:010d}.jpg".format(index), rgb_image)
    return True

def compressed_image_callback(msg):
  """If saving the compressed image worked out, write the log information and increment index."""
  global index, rgb_cb_rate, rgb_cb_ts, rgb_write_ts, rgb_write_rate
  # if rgb_cb_ts != 0: rgb_cb_rate.append(time.time()-rgb_cb_ts)
  rgb_cb_ts = time.time()
  if process_rgb_compressed(msg, index):
    if index > skip_first: write_info('RGB', index)
    index+=1
    # if rgb_write_ts != 0: rgb_write_rate.append(time.time()-rgb_write_ts)
    rgb_write_ts=time.time()

def process_rgb(msg, index, saving_location=None):
  """If ready-state: go from serial to rgb image and save it."""
  if (not ready) or finished: return False
  if not saving_location: 
    saving_location=data_location
  try:
    # Convert your ROS Image message to OpenCV2
    rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError, e:
    print(e)
  else:
    # Save your OpenCV2 image as a jpeg 
    if index > skip_first: 
      # print('[create_dataset.py]: {2}: write RGB image {1} to {0}'.format(data_location, index, rospy.get_time()))
      cv2.imwrite(saving_location+"/RGB/{:010d}.jpg".format(index), rgb_image)
    return True

def image_callback(msg, camera_type='straight'):
  """If saving the image worked out, write the log information and increment index."""
  global index, rgb_cb_rate, rgb_cb_ts, rgb_write_ts, rgb_write_rate, index_dict
  # print "[create_dataset]: {2} : received image. index: {0} skip_first: {1}".format(index, skip_first, rospy.get_time())
  # if rgb_cb_ts != 0: rgb_cb_rate.append(time.time()-rgb_cb_ts)
  rgb_cb_ts = time.time()
  # pause simulator
  if camera_type=='straight' in globals(): 
    pause_physics_client(EmptyRequest())
  
  # print("index straight: {0}, index_dict {2}: {1}".format(index, [index_dict[cam] for cam in sorted(index_dict.keys())], sorted(index_dict.keys())))
  if camera_type != 'straight':
    if process_rgb(msg, index_dict[camera_type], saving_location=data_location+'_'+camera_type):
      if index_dict[camera_type] > skip_first: 
        write_info('RGB', index_dict[camera_type], saving_location=data_location+'_'+camera_type, direction=camera_type)
      index_dict[camera_type]+=1
  else:
    if process_rgb(msg, index):
      if index > skip_first: 
        write_info('RGB', index)
      index+=1
    # if rgb_write_ts != 0: rgb_write_rate.append(time.time()-rgb_write_ts)
    rgb_write_ts=time.time()
  # resume simulator
  if camera_type=='straight' in globals(): 
    unpause_physics_client(EmptyRequest())

def process_depth(msg, index):
  """If ready-state: go from serial to depth image and save it."""
  if (not ready) or finished: return False
  try:
    # Convert your ROS Image message to OpenCV2
    im = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')#gets float of 32FC1 depth image
  except CvBridgeError as e:
    print(e)
  else:
    im=im*(1/5.*255) # scale from 0:5 to 0:255
    if index > skip_first: 
      # print('[create_dataset.py]: {2}: write Depth image {1} to {0}'.forma, index, rospy.get_time()))
      cv2.imwrite(data_location+"/Depth/{:010d}.jpg".format(index), im.astype(np.int))
    return True

def depth_callback(msg):
  """If saving the image worked out, write the log information and increment index."""
  global index  
  if process_depth(msg, index):
    if index > skip_first: write_info('Depth', index)
    index+=1

def scan_callback(msg):
    """Preprocess serial scan: Preprocess and save serial scan in last_scan field. """
    global last_scan
    if (not ready) or finished: return
    # Clip at 5m and put zeros at 5m 
    last_scan=[5 if r > 5 or r==0 else r for r in msg.ranges]

def control_callback(data):
  """Save /cmd_vel info in last_control field."""
  global last_control
  if (not ready) or finished: return  
  last_control=[data.linear.x,
      data.linear.y,
      data.linear.z,
      data.angular.x,
      data.angular.y,
      data.angular.z]

def supervised_control_callback(data):
  """Save /supervised_vel info in last_supervised_control field."""
  global last_supervised_control
  if (not ready) or finished: return  
  last_supervised_control=[data.linear.x,
      data.linear.y,
      data.linear.z,
      data.angular.x,
      data.angular.y,
      data.angular.z]
  
def gt_callback(data):
  """Save current position in last_position.
  Save quaternion of current orientation in T_pg.
  Save the odometry (6D) defining transformation between current frame and previous frame."""
  global last_position, last_odom, T_pg
  if (not ready) or finished: return  
  quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
  # orientation of current frame relative to global frame
  T_cg = tf.transformations.quaternion_matrix(quaternion)
  
  r,p,yw= tf.transformations.euler_from_matrix(T_cg, 'rxyz')
  
  last_position = [data.pose.pose.position.x,
    data.pose.pose.position.y,
    data.pose.pose.position.z,
    r,
    p,
    yw]
  # add position of global frame
  T_cg[0:3,3]=last_position[0:3]
  if len(T_pg)!=0:
    # if previous transformation in global frame is there,
    # calculate the local transformation between the two poses relative to the global frame.
    # get the inverse of the previous pose relative to the global frame
    i_T_pg = tf.transformations.inverse_matrix(T_pg)
    # concatenate the transformation of current frame to global frame
    # and from global frame to previous frame to get transformation from
    # current frame to previous frame as the 6d odometry.
    T_cp = tf.transformations.concatenate_matrices(i_T_pg, T_cg)
    # extract roll pitch yaw and position
    r,p,yw = tf.transformations.euler_from_matrix(T_cp, 'rxyz')
    x,y,z = T_cp[0:3,3]
    # save the odometry
    last_odom = [x,y,z,r,p,yw]
  T_pg = copy.deepcopy(T_cg) 

# def image_callback_left_30(msg, sloc):
#   global index_left_30  
#   if process_rgb(msg, sloc, index_left_30):
#     write_info('RGB', sloc, index_left_30)
#     index_left_30+=1
# def image_callback_left_60(msg, sloc):
#   global index_left_60  
#   if process_rgb(msg, sloc, index_left_60):
#     write_info('RGB', sloc, index_left_60)
#     index_left_60+=1
# def image_callback_right_30(msg, sloc):
#   global index_right_30  
#   if process_rgb(msg, sloc, index_right_30):
#     write_info('RGB', sloc, index_right_30)
#     index_right_30+=1
# def image_callback_right_60(msg, sloc):
#   global index_right_60  
#   if process_rgb(msg, sloc, index_right_60):
#     write_info('RGB', sloc, index_right_60)
#     index_right_60+=1
  

def ready_callback(msg):
  """ callback function that makes create_ds start saving images and toggles ready"""
  global ready, finished, data_location, index, last_supervised_control, last_control, last_scan, last_gt, last_position, last_odom, T_pg, index_dict  
  
  if not ready and finished:  
    print("[create_dataset]: Start saving images.")
    ready=True
    finished = False

    # update data_location and make dires
    if rospy.has_param('data_location'):
      loc=rospy.get_param('data_location')
      if loc[0]=='/':
        data_location=loc
      else:
        data_location=os.environ['HOME']+'/pilot_data/'+loc
      if not os.path.exists(data_location+'/RGB'): os.makedirs(data_location+'/RGB')
      if not os.path.exists(data_location+'/Depth'): os.makedirs(data_location+'/Depth')
      index=0
      last_supervised_control=[0,0,0,0,0,0]
      last_control=[0,0,0,0,0,0]
      last_scan=[]
      last_gt=[0,0,0]
      last_position=[]
      last_odom=[]
      T_pg=[] # transformation of previous pose in global coordinates
      if recovery:
        for direction in ['left','right']:      
          if not os.path.exists(data_location+'_'+direction+'/RGB'): os.makedirs(data_location+'_'+direction+'/RGB')
          if not os.path.exists(data_location+'_'+direction+'/Depth'): os.makedirs(data_location+'_'+direction+'/Depth')
          index_dict={'left':0,'right':0} #Extra indices for recovery cameras


    print('[create_dataset]: ready: {0}: {1}'.format(rospy.get_time(), data_location))

def finished_callback(msg):
  """ callback function that makes create_ds stop and toggles finished"""
  global ready, finished
  if ready and not finished:
    ready=False
    finished = True
    # print('[create_dataset]: finished: {0}. RGB callback rate: {1:0.3f}({2:0.2f}) and RGB write rate: {3:0.3f}({4:0.2f})'.format(rospy.get_time(), 
    #                                                                                                           np.mean(rgb_cb_rate),
    #                                                                                                           np.var(rgb_cb_rate),
    #                                                                                                           np.mean(rgb_write_rate),
    #                                                                                                           np.var(rgb_write_rate)))

def write_info(image_type, index, saving_location=None, direction='straight'):
  """For each image (lowest rate) save information regarding the position, control or scan readings."""
  if (not ready) or finished: return
  if not saving_location:
    saving_location=data_location
  # first copy all info in order to have the update closest to saving previous frame.
  supervised_control = last_supervised_control[:] # copy
  control=last_control[:] #copy
  # adjust control according to direction
  if direction != 'straight':
    if direction=='left':
      control[5] = control[5]-0.9 
    elif direction=='right':
      control[5] = control[5]+0.9 
    else:
      print("[create_dataset.py]: direction {} is unknown.".format(direction))
  odom=last_odom[:]
  position=last_position[:]
  scan=last_scan[:]
  # open and append information
  with open(saving_location+'/supervised_info.txt','a') as supervised_controlfile:
    supervised_controlfile.write("{0:010d} {1[0]} {1[1]} {1[2]} {1[3]} {1[4]} {1[5]}\n".format(index, supervised_control))
  with open(saving_location+'/control_info.txt','a') as controlfile:
    controlfile.write("{0:010d} {1[0]} {1[1]} {1[2]} {1[3]} {1[4]} {1[5]}\n".format(index, control))
  with open(saving_location+'/position_info.txt','a') as positionfile:
    positionfile.write("{0:010d} {1}\n".format(index, str(position)))
  with open(saving_location+'/odom_info.txt','a') as odomfile:
    odomfile.write("{0:010d} {1}\n".format(index, str(odom)))
  with open(saving_location+'/images.txt','a') as imagesfile:
    imagesfile.write("{3}s:{4}ns {0}/{1}/{2:010d}.jpg\n".format(saving_location, image_type, index, rospy.get_rostime().secs, rospy.get_rostime().nsecs))
  with open(saving_location+'/scan.txt','a') as scanfile:
    scanfile.write("{0:010d} {1}\n".format(index, str(scan)))
  # print("wrote: ctr: {0} for index {1} of cam {2}".format(control, index, direction))

if __name__=="__main__":
  rospy.init_node('create_dataset', anonymous=True)
  
  # set core functionality
  rospy.Subscriber('/createds_start', Empty, ready_callback)
  rospy.Subscriber('/createds_stop', Empty, finished_callback)

  # setup saving location in ~/pilot_data
  if rospy.has_param('data_location'):
    loc=rospy.get_param('data_location')
    if loc[0]=='/':
      data_location=loc
    else:
      data_location=os.environ['HOME']+'/pilot_data/'+loc
  if not data_location:
    print '[create dataset]: Found no saving location: {}'.format(data_location)
    sys.exit(2)
  print '[create dataset]: Saving location: {}'.format(data_location)
  if not os.path.exists(data_location+'/RGB'): os.makedirs(data_location+'/RGB')
  if not os.path.exists(data_location+'/Depth'): os.makedirs(data_location+'/Depth')

  # initialize info files with a header defining information saved
  with open(data_location+'/control_info.txt','w') as controlfile:
    controlfile.write("Actual control \n index | linear velocity x, y, z, angular velocity x, y, z\n")
  with open(data_location+'/supervised_info.txt','w') as supervised_controlfile:
    supervised_controlfile.write("Supervised control \n index | linear velocity x, y, z, angular velocity x, y, z\n")
  with open(data_location+'/position_info.txt','w') as positionfile:
    positionfile.write("Position in global frame\n index | x | y | z | \n")
  with open(data_location+'/odom_info.txt','w') as odomfile:
    odomfile.write("Odometry between two frames \n index | x | y | z | roll | pitch | yaw |\n")
  with open(data_location+'/scan.txt','w') as scanfile:
    scanfile.write("Read out LiDAR scan\n index | 360 degrees scan readings clipped at 5m |\n")
  
  # initialize subscribers
  if rospy.has_param('control'): rospy.Subscriber(rospy.get_param('control'), Twist, control_callback)
  rospy.Subscriber('/supervised_vel', Twist, supervised_control_callback)
  if rospy.has_param('gt_info'): rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_callback)


  if rospy.has_param('rgb_image'): 
    if 'compressed' in rospy.get_param('rgb_image'): 
      rospy.Subscriber(rospy.get_param('rgb_image'), CompressedImage, compressed_image_callback, queue_size = 20)  
    else:
      rospy.Subscriber(rospy.get_param('rgb_image'), Image, image_callback, queue_size = 20)  
  if rospy.has_param('depth_image'): 
    if 'scan' in rospy.get_param('depth_image'):
      rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, scan_callback)
    else:
      rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_callback)


  if rospy.has_param('recovery'):
    recovery = rospy.get_param('recovery')
  if recovery:
    for direction in ['left','right']:
      rospy.Subscriber(rospy.get_param('rgb_image_'+direction), Image, image_callback, callback_args=direction, queue_size = 20)
      if not os.path.exists(data_location+'_'+direction+'/RGB'): os.makedirs(data_location+'_'+direction+'/RGB')
      if not os.path.exists(data_location+'_'+direction+'/Depth'): os.makedirs(data_location+'_'+direction+'/Depth')

    

    
  #   callbacks={'left':{'30':image_callback_left_30,'60':image_callback_left_60},'right':{'30':image_callback_right_30,'60':image_callback_right_60}}
  #   callbacks_depth={'left':{'30':depth_callback_left_30,'60':depth_callback_left_60},'right':{'30':depth_callback_right_30,'60':depth_callback_right_60}}
  #   for d in ['left','right']:
  #     for c in ['30','60']:
  #       rospy.Subscriber(re.sub(r"kinect","kinect_"+d+"_"+c,rospy.get_param('rgb_image')), Image, callbacks[d][c],(data_location+'_'+d+'_'+c))
  #       rospy.Subscriber(re.sub(r"kinect","kinect_"+d+"_"+c,rospy.get_param('depth_image')), Image, callbacks_depth[d][c],(data_location+'_'+d+'_'+c))

  # spin() simply keeps python from exiting until this node is stopped	
  rospy.spin()


#### MORE CALLBACKS FOR DIFFERENT SENSORS:



# def depth_callback_left_30(msg, sloc):
#   global index_left_30  
#   if process_depth(msg, sloc, index_left_30):
#     write_info('Depth', sloc, index_left_30)
#     index_left_30+=1
# def depth_callback_left_60(msg, sloc):
#   global index_left_60  
#   if process_depth(msg, sloc, index_left_60):
#     write_info('Depth', sloc, index_left_60)
#     index_left_60+=1
# def depth_callback_right_30(msg, sloc):
#   global index_right_30  
#   if process_depth(msg, sloc, index_right_30):
#     write_info('Depth', sloc, index_right_30)
#     index_right_30+=1
# def depth_callback_right_60(msg, sloc):
#   global index_right_60  
#   if process_depth(msg, sloc, index_right_60):
#     write_info('Depth', sloc, index_right_60)
#     index_right_60+=1

# def odometry_callback(data):
#   global last_gt
#   last_gt=[data.pose.pose.orientation.x,
#       data.pose.pose.orientation.y,
#       data.pose.pose.orientation.z]
#   #print('received odo: ',last_gt)
  
# def imu_callback(data):
#   global last_gt
#   last_gt=[data.orientation.x,
#       data.orientation.y,
#       data.orientation.z,
#       data.orientation.w,
#       data.angular_velocity.x,
#       data.angular_velocity.y,
#       data.angular_velocity.z,
#       data.linear_acceleration.x,
#       data.linear_acceleration.y,
#       data.linear_acceleration.z]
