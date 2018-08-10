#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
import time
import sys, select, tty, os, os.path
import numpy as np

# import scipy.misc as sm

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Instantiate CvBridge
bridge = CvBridge()


real=False
ready=False
finished=False
disp_depth_im=False #display depth image instead of array

count=0
saving_location='/tmp'
save_images=False


fig=plt.figure(figsize=(15,5))
plt.title('Predictions')


target_depth = np.zeros((120,240))
predicted_depth = np.zeros((3,3*55,3*74))

def combine_target_prediction():
  big_image = np.ones((200,250+predicted_depth.shape[0]*230))  
  
  for i in range(predicted_depth.shape[0]):
    d=5
    big_image[20:20+predicted_depth.shape[1], 10+target_depth.shape[1]+(i+1)*d+i*predicted_depth.shape[2]:10+target_depth.shape[1]+(i+1)*d+(1+i)*predicted_depth.shape[2]] = predicted_depth[i,:,:]
    
  if target_depth.sum()!= 0:
    big_image[35:35+target_depth.shape[0], 10:10+target_depth.shape[1]] = target_depth[:,:]

  # if save_images:
  #   cv2.imwrite(saving_location+'/'+'{0:010d}.jpg'.format(count),big_image)
  #   count+=1

  return big_image


implot=plt.imshow(combine_target_prediction(),animated=True, cmap='winter')

def update_im(*args):
  image=combine_target_prediction()
  implot.set_array(image)
  return implot,

anim=animation.FuncAnimation(fig,update_im)
  
def ready_callback(msg):
  global ready, finished
  if not ready:
    print('[show_depth] ready')
    ready = True
    finished = False

def finished_callback(msg):
  global finished, ready, anim
  if not finished:
    print('finished')
    finished = True
    ready = False
    anim.event_source.stop()
    plt.close(fig)
    
def target_callback(data):
  global target_depth
  # if not ready: return
  try:
    # Convert your ROS Image message to OpenCV2
    im = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')#gets float of 32FC1 depth image
  except CvBridgeError as e:
    print(e)
  else:
    im = im[::3,::3]
    shp = im.shape
    im=np.asarray([ e*1.0 if not np.isnan(e) else 0. for e in im.flatten()]).reshape(shp)
    im = im*1/5.
    target_depth = im[:,:]
    # update()
    
def predicted_callback(data):
  global predicted_depth
  # if not ready: return
  img = data.data
  # check if the predicted depth contains probabilities instead:
  if len(img.flatten()) < 10 and sum(img.flatten()) != 0: #in case only or less than 10 values are send these are most likely to be continuous output values.
    for i in range(len(img.flatten())):
      predicted_depth[i,:,:] = img.flatten()[i]*1/max(img.flatten())
  else:
    try:
      img = img.reshape(-1,55,74)
    except:
      return
      # img = predicted_depth.reshape(55,74)
    img = img*1/5.
    n=3
    img = np.kron(img, np.ones((n,n)))
    predicted_depth = img[:,:,:]
  # update()


  
if __name__=="__main__":
  rospy.init_node('show_depth', anonymous=True)
  # r = rospy.Rate(10) 
  rospy.Subscriber('/depth_prediction', numpy_msg(Floats), predicted_callback, queue_size = 1)

  if rospy.has_param('ready'):
    rospy.Subscriber(rospy.get_param('ready'), Empty, ready_callback)
  if rospy.has_param('finished'):
    rospy.Subscriber(rospy.get_param('finished'), Empty, finished_callback)
  if rospy.has_param('depth_image'):
    rospy.Subscriber(rospy.get_param('depth_image'), Image, target_callback, queue_size = 1)
  
  plt.show()

  rospy.spin()
