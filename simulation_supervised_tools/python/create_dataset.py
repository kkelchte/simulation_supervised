#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import numpy as np
import copy
import tf
import sys, select, tty, os, os.path, re

# Instantiate CvBridge
bridge = CvBridge()

saving_location = '/tmp/unknown'
direction = 'none'
coltype = 'none'
save_images = True
index=0
index_left_30=0
index_left_60=0
index_right_30=0
index_right_60=0
# index_depth=0
last_control=[0,0,0,0,0,0]
last_imu=[0,0,0]
last_position=[]
last_odom=[]
ready=False
finished=True
start_time=0
delay_evaluation = 3
rgb_image = None #np.zeros((360,640,3))
recovery = False
T_pg = []
# saving = False # token to avoid overwriting of rgb_image while saving

def write_info(image_type, sloc, index):
  if not ready or (rospy.get_time()-start_time) < delay_evaluation: return
  # parse compensation:
  dr=0 #direction
  dg=0 #degrees
  if sloc.find("right")!=-1:
    dr=+1
  if sloc.find("left")!=-1:
    dr=-1
  if sloc.find("60")!=-1:
    dg=60
  if sloc.find("30")!=-1:
    dg=30
  compensation=dr*dg/60.
  control=last_control[:] #copy
  control[5]=compensation+control[5] # compensate
  # print("sloc: {0}: dr: {1}, dg: {2}, control[5]: {3} = last_control[5] ({4}) + compensation ({5})".format(sloc, dr, dg, control[5], last_control[5], compensation))
  # with open(sloc+'/meta_info.txt','a') as metafile:
  #   metafile.write("{0:010d} {1}\n".format(index, str(last_imu)[1:-1]))
  with open(sloc+'/control_info.txt','a') as controlfile:
    controlfile.write("{0:010d} {1[0]} {1[1]} {1[2]} {1[3]} {1[4]} {1[5]}\n".format(index, control))
  with open(sloc+'/position_info.txt','a') as positionfile:
    positionfile.write("{0:010d} {1}\n".format(index, str(last_position)[1:-1]))
  with open(sloc+'/odom_info.txt','a') as odomfile:
    odomfile.write("{0:010d} {1}\n".format(index, str(last_odom)[1:-1]))
  with open(sloc+'/images.txt','a') as imagesfile:
    imagesfile.write("{0}/{1}/{2:010d}.jpg\n".format(sloc, image_type, index))
  
def process_rgb(msg, sloc, index):
  if (not ready) or finished or ((rospy.get_time()-start_time) < delay_evaluation): return False
  try:
    # Convert your ROS Image message to OpenCV2
    rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError, e:
    print(e)
  else:
    # Save your OpenCV2 image as a jpeg 
    print('write RGB image {1} to {0}'.format(sloc, index))
    cv2.imwrite(sloc+"/RGB/{:010d}.jpg".format(index), rgb_image)
    return True  

def image_callback(msg, sloc):
  global index  
  # print('received image for: {}'.format(sloc))
  if process_rgb(msg, sloc, index):
    write_info('RGB', sloc, index)
    index+=1
def image_callback_left_30(msg, sloc):
  global index_left_30  
  if process_rgb(msg, sloc, index_left_30):
    write_info('RGB', sloc, index_left_30)
    index_left_30+=1
def image_callback_left_60(msg, sloc):
  global index_left_60  
  if process_rgb(msg, sloc, index_left_60):
    write_info('RGB', sloc, index_left_60)
    index_left_60+=1
def image_callback_right_30(msg, sloc):
  global index_right_30  
  if process_rgb(msg, sloc, index_right_30):
    write_info('RGB', sloc, index_right_30)
    index_right_30+=1
def image_callback_right_60(msg, sloc):
  global index_right_60  
  if process_rgb(msg, sloc, index_right_60):
    write_info('RGB', sloc, index_right_60)
    index_right_60+=1

def process_depth(msg, sloc, index):
  if (not ready) or finished or ((rospy.get_time()-start_time) < delay_evaluation): return False
  try:
    # Convert your ROS Image message to OpenCV2
    im = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')#gets float of 32FC1 depth image
  except CvBridgeError as e:
    print(e)
  else:
    im=im*1/5.*255
    print('write Depth image {1} to {0}'.format(sloc, index))
    cv2.imwrite(sloc+"/Depth/{:010d}.jpg".format(index), im.astype(np.int))
    return True

def depth_callback(msg, sloc):
  global index  
  # print('received image for: {}'.format(sloc))
  if process_depth(msg, sloc, index):
    write_info('Depth', sloc, index)
    index+=1
def depth_callback_left_30(msg, sloc):
  global index_left_30  
  if process_depth(msg, sloc, index_left_30):
    write_info('Depth', sloc, index_left_30)
    index_left_30+=1
def depth_callback_left_60(msg, sloc):
  global index_left_60  
  if process_depth(msg, sloc, index_left_60):
    write_info('Depth', sloc, index_left_60)
    index_left_60+=1
def depth_callback_right_30(msg, sloc):
  global index_right_30  
  if process_depth(msg, sloc, index_right_30):
    write_info('Depth', sloc, index_right_30)
    index_right_30+=1
def depth_callback_right_60(msg, sloc):
  global index_right_60  
  if process_depth(msg, sloc, index_right_60):
    write_info('Depth', sloc, index_right_60)
    index_right_60+=1

def control_callback(data):
  global last_control
  if direction == 'right':
    # right camera --> turn left --> +1
    last_control=[1.3,0,0,0,0,1]
  elif direction == 'left':
    # left camera --> turn right --> -1
    last_control=[1.3,0,0,0,0,-1]
  elif direction == 'straight':
    # straight
    last_control=[1.3,0,0,0,0,0]
  else :
    last_control=[data.linear.x,
        data.linear.y,
        data.linear.z,
        data.angular.x,
        data.angular.y,
        data.angular.z]
  
  #print('received control:', data.linear.x,', ',data.linear.y,', ',data.linear.z)

# def odometry_callback(data):
#   global last_imu
#   last_imu=[data.pose.pose.orientation.x,
# 	    data.pose.pose.orientation.y,
# 	    data.pose.pose.orientation.z]
#   #print('received odo: ',last_imu)
  
# def imu_callback(data):
#   global last_imu
#   last_imu=[data.orientation.x,
# 	    data.orientation.y,
# 	    data.orientation.z,
# 	    data.orientation.w,
# 	    data.angular_velocity.x,
# 	    data.angular_velocity.y,
# 	    data.angular_velocity.z,
# 	    data.linear_acceleration.x,
# 	    data.linear_acceleration.y,
# 	    data.linear_acceleration.z]
  
def gt_callback(data):
  global last_position, last_odom, T_pg
  quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
  last_position = [data.pose.pose.position.x,
    data.pose.pose.position.y,
    data.pose.pose.position.z]
  # euler = tf.transformations.euler_from_quaternion(quaternion)
  T_cg = tf.transformations.quaternion_matrix(quaternion) # orientation of current frame relative to global frame
  T_cg[0:3,3]=last_position
  # print 'current: ',str(euler[2]), str(T_cg[0,3]),str(T_cg[1,3])
  if len(T_pg)!=0:
    i_T_pg = tf.transformations.inverse_matrix(T_pg)
    # euler = tf.transformations.euler_from_matrix(i_T_pg, 'rxyz')
    # print 'inverse prev: ',str(euler[2]), str(i_T_pg[0,3]),str(i_T_pg[1,3])
    T_cp = tf.transformations.concatenate_matrices(i_T_pg, T_cg)
    r,p,yw = tf.transformations.euler_from_matrix(T_cp, 'rxyz')
    x,y,z = T_cp[0:3,3]
    last_odom = [x,y,z,r,p,yw]
    # print 'odom: ',str(last_odom)
  T_pg = copy.deepcopy(T_cg) 

  
def ready_callback(msg):
  global ready, start_time, finished
  if not ready and finished:
    ready=True
    finished = False
    start_time=rospy.get_time()
    # print('evaluate start: ', start_time)

def finished_callback(msg):
  global ready, start_time, finished
  if ready and not finished:
    ready=False
    finished = True
    start_time=0
    # print('evaluate start: ', start_time)

if __name__=="__main__":
  if rospy.has_param('delay_evaluation'):
    delay_evaluation=rospy.get_param('delay_evaluation')
  
  rospy.init_node('create_dataset', anonymous=True)
  
  if rospy.has_param('control'):
    rospy.Subscriber(rospy.get_param('control'), Twist, control_callback)
  rospy.Subscriber('/ground_truth/state', Odometry, gt_callback)
  if rospy.has_param('ready'):
    rospy.Subscriber(rospy.get_param('ready'), Empty, ready_callback)
  if rospy.has_param('finished'):
    rospy.Subscriber(rospy.get_param('finished'), Empty, finished_callback)
  if rospy.has_param('overtake'):
    rospy.Subscriber(rospy.get_param('overtake'), Empty, finished_callback)

  save_images = True
  if rospy.has_param('save_images'):
    if not rospy.get_param('save_images'): 
      save_images=bool(rospy.get_param('save_images')!='false')
      print('not saving images') 
      sys.exit(0)
  
  if rospy.has_param('direction'):
    direction=rospy.get_param('direction')
  print '--> create dataset: ',direction
  
  if rospy.has_param('saving_location'):
    loc=rospy.get_param('saving_location')
    if loc[0]=='/':
      saving_location=loc
    else:
      saving_location=os.printenv('HOME')+'/pilot_data/'+loc
  print 'saving_location ',saving_location
  if not os.path.exists(saving_location+'/RGB'):
      os.makedirs(saving_location+'/RGB')
  if not os.path.exists(saving_location+'/Depth'):
      os.makedirs(saving_location+'/Depth')
  
  if rospy.has_param('rgb_image'):
    rospy.Subscriber(rospy.get_param('rgb_image'), Image, image_callback, saving_location)
  if rospy.has_param('depth_image'):
    rospy.Subscriber(rospy.get_param('depth_image'), Image, depth_callback, saving_location, queue_size = 1)
  
  if rospy.has_param('recovery'):
    recovery = rospy.get_param('recovery')
  if recovery:
    callbacks={'left':{'30':image_callback_left_30,'60':image_callback_left_60},'right':{'30':image_callback_right_30,'60':image_callback_right_60}}
    callbacks_depth={'left':{'30':depth_callback_left_30,'60':depth_callback_left_60},'right':{'30':depth_callback_right_30,'60':depth_callback_right_60}}
    for d in ['left','right']:
      for c in ['30','60']:
        rospy.Subscriber(re.sub(r"kinect","kinect_"+d+"_"+c,rospy.get_param('rgb_image')), Image, callbacks[d][c],(saving_location+'_'+d+'_'+c))
        rospy.Subscriber(re.sub(r"kinect","kinect_"+d+"_"+c,rospy.get_param('depth_image')), Image, callbacks_depth[d][c],(saving_location+'_'+d+'_'+c))
        # create necessary directories
        if not os.path.exists(saving_location+'_'+d+'_'+c+'/RGB'):
            os.makedirs(saving_location+'_'+d+'_'+c+'/RGB')
        if not os.path.exists(saving_location+'_'+d+'_'+c+'/Depth'):
            os.makedirs(saving_location+'_'+d+'_'+c+'/Depth')
      
  print('saving location: {}'.format(saving_location))
  print('save_images: {}'.format(save_images))
  # spin() simply keeps python from exiting until this node is stopped	
  rospy.spin()


