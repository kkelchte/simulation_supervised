#!/usr/bin/env python
import rospy
# OpenCV2 for saving an image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
import time
import sys, select, tty, os, os.path
import numpy as np

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import matplotlib.pyplot as plt
import matplotlib.animation as animation

#--------------------------------------------------------------------------------------------------------------
#
# Show scan prediction displays the predicted and the ground truth scan information.
# This node will only display if 2 sources (scan, predicted scan) is published.
# The node will save the images if save_images param is set true and there is a non-zero data_location.
#
#--------------------------------------------------------------------------------------------------------------

# Instantiate CvBridge
bridge = CvBridge()

# Initialize data saving parameters
count=0
data_location='/tmp'
save_images=False

# Scan settings
field_of_view = 104 # FOV in degrees
clip_distance = 2 # don't care about everything further than 5m away.
smooth_x = 4 # smooth over 4 neighboring bins

# Global fields
target_scan = None
# predicted_scan = None
predicted_scans = {}
prediction_barcollections={}

action=0

# Initialize plotting figure
# fig=plt.figure(figsize=(15,5))
# plt.title('Scan Predictions')
# target_barcollection=plt.bar(np.arange(-field_of_view/2, field_of_view/2, 0.5*smooth_x),[clip_distance if k%2==1 else 0 for k in range(int(field_of_view/(0.5*smooth_x))) ],align='center',color='blue',width=0.4)
# predicted_barcollection=plt.bar(np.arange(-field_of_view/2, field_of_view/2, 0.5*smooth_x),[clip_distance if k%2==0 else 0 for k in range(int(field_of_view/(0.5*smooth_x))) ],align='center',color='red',width=0.4)
fig,ax = plt.subplots(5,1,sharex=True, sharey=False, figsize=(20,10))
# fig,ax = plt.subplots(4,1,sharex=True, sharey=True, figsize=(20,10))
target_barcollection = ax[0].bar(np.arange(-field_of_view/2, field_of_view/2, smooth_x),[clip_distance for k in range(int(field_of_view/(smooth_x))) ],align='center',color='red',width=0.5)
ax[0].set_title('SCAN')
prediction_barcollections[1] = ax[1].bar(np.arange(-field_of_view/2, field_of_view/2, smooth_x),[clip_distance for k in range(int(field_of_view/(smooth_x))) ],align='center',color='blue',width=0.5)
ax[1].set_title('LEFT (action: 1)')
prediction_barcollections[0] = ax[2].bar(np.arange(-field_of_view/2, field_of_view/2, smooth_x),[clip_distance for k in range(int(field_of_view/(smooth_x))) ],align='center',color='blue',width=0.5)
ax[2].set_title('STRAIGHT (action: 0)')
prediction_barcollections[-1] = ax[3].bar(np.arange(-field_of_view/2, field_of_view/2, smooth_x),[clip_distance for k in range(int(field_of_view/(smooth_x))) ],align='center',color='blue',width=0.5)
ax[3].set_title('RIGHT. (action: -1)')
diff_barcollections = ax[4].bar(np.arange(-field_of_view/2, field_of_view/2, smooth_x),[0 for k in range(int(field_of_view/(smooth_x))) ],align='center',color='blue',width=0.5)
ax[4].set_title('LEFT - RIGHT')

def animate(n):
  # only animate if all fields are filled.
  # ax[0].set_title("Action: {} \n SCAN".format(action))

  if target_scan:
    # put even slots with target scan
    for i, b in enumerate(target_barcollection): b.set_height(min(target_scan[i], clip_distance))
      # if i%2==1:
      #   b.set_height(min(target_scan[int(i/2)], clip_distance))
      # else:
      #   b.set_height=0
    # put odd slots with predicted scan
  # if predicted_scan:
  #   for i, b in enumerate(predicted_barcollection):
  #     if i%2==0:
  #       b.set_height(min(predicted_scan[int(i/2)], clip_distance))
  #     else:
  #       b.set_height=0
  if len(predicted_scans)==3:
    for a in [1,0,-1]:
      for i, b in enumerate(prediction_barcollections[a]): 
        if action == a: 
          b.set_color('green')
        else:
          b.set_color('blue')
        b.set_height(min(predicted_scans[a][i], clip_distance))
  if len(predicted_scans)==3:
    for i, b in enumerate(diff_barcollections): b.set_height(predicted_scans[1][i]-predicted_scans[-1][i]) 

def target_callback(data):
  """ Preprocess target scan (180degrees):
  - clip values at clip_distance
  - clip at -field_of_view/2:field_of_view
  - set zeros to nans
  - smooth over smooth_x neighboring range values
  return clean scan
  """
  global target_scan
  # clip left field_of_view/2 degree range from 0:field_of_view/2  reversed with right field_of_view/2 degree range from the last field_of_view/2 :
  data.ranges=list(reversed(data.ranges[:field_of_view/2]))+list(reversed(data.ranges[-field_of_view/2:]))
  # clip at clip_distance m and make 'broken' 0 readings also clip_distance 
  ranges=[min(r,clip_distance) if r!=0 else np.nan for r in data.ranges]
  # smooth over smooth_x bins and save in target ranges
  target_scan = [np.nanmean(ranges[i*smooth_x:i*smooth_x+smooth_x]) for i in range(int(len(ranges)/smooth_x))]
    
def predicted_callback(data):
  global predicted_scans,action
  # action=data.data[0]
  # scans=data.data[1:]
  scans=data.data[:]

  if len(scans) != 3*field_of_view/smooth_x:
    print "[show_scan_prediction]: length of scan is not 3 so don't show depth prediction."
  else:
    min_scans=[]
    for i, a in enumerate([-1,0,1]):
      predicted_scans[a]=list(scans[i*int(field_of_view/smooth_x):i*int(field_of_view/smooth_x)+int(field_of_view/smooth_x)])
      min_scans.append(min(predicted_scans[a]))
    
    action=(np.argmax(min_scans)-1)
    # min_scans=[min(predicted_scans[1]), min(predicted_scans[-1])]
    # action=2*np.argmax(min_scans)-1
  return
#   # if not ready: return
#   img = scans
#   # check if the predicted depth contains probabilities instead:
#   if len(img.flatten()) < 10 and sum(img.flatten()) != 0: #in case only or less than 10 values are send these are most likely to be continuous output values.
#     for i in range(len(img.flatten())):
#       predicted_depth[i,:,:] = img.flatten()[i]*1/max(img.flatten())
#   else:
#     try:
#       img = img.reshape(-1,55,74)
#     except:
#       return
#       # img = predicted_depth.reshape(55,74)
#     img = img*1/5.
#     n=3
#     img = np.kron(img, np.ones((n,n)))
#     predicted_depth = img[:,:,:]
#   # update()


def cleanup():
  """Get rid of the animation on shutdown"""
  plt.close(fig)
  plt.close()

  
if __name__=="__main__":
  rospy.init_node('show_scan', anonymous=True)
  
  rospy.Subscriber('/depth_prediction', numpy_msg(Floats), predicted_callback, queue_size = 1)

  if rospy.has_param('depth_image') and 'scan' in rospy.get_param('depth_image'):
    rospy.Subscriber(rospy.get_param('depth_image'), LaserScan, target_callback, queue_size = 1)
      
  if rospy.has_param('graphics'):
    if rospy.get_param('graphics'):
      print("[show_scan_prediction]: showing graphics.")
      anim=animation.FuncAnimation(fig,animate)
      plt.show()
  rospy.on_shutdown(cleanup)

  rospy.spin()
