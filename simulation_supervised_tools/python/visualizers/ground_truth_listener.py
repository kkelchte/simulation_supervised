#!/usr/bin/env python
import rospy
import numpy as np
from numpy.linalg import inv
import os, sys, time

import tf

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.animation as animation
import matplotlib.patches as patches

from std_msgs.msg import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import subprocess, shlex


#--------------------------------------------------------------------------------------------------------------------------------
#
# Listen to current ground truth positions 
# in order to create evaluation plots
#
#--------------------------------------------------------------------------------------------------------------------------------

turtle=False
size = (200,200,3)
img = np.ones(size)
img_type = "unknown"
last_position = []
ready = False
finished = True
transformations={'unknown':(2,1,-2,1),
    'corridor':(5,300,-5,400),
    'different_corridor':(5,300,-5,400),
    'forest':(6.5, 438.0, -6.6, 418.0),
    'canyon':(15.5, 424.0, -10.6, 780),
    'sandbox':(38.676923076923075, 438.0, -39.876923076923077, 418.0),
    'esatv1':(16.947783251231524, 63.724137931034484, -16.548448275862071, 590.18620689655177),
    'esatv2':(17.22058089465456, 1065.4257425742574, -17.138477795147935, 843.90099009900985),
    'forest_real':(17.2170726,785,-17.07010944,487),
    # 'esatv3':(0,25.76,10,-22,0,4)}
    # 'esatv3':(0,25.76,10,-22,0,4)}
    # 'esatv3':(-22.03,0,88.13,0,25.76,257.6)}
    'esatv3':(-20.66,0,82.625,0,20.76,207.57)}

    # 'canyon':(15.5, 424.0, -10.6, 809),
    # 'esat_v2':(16.321360645256139, 1006.9433962264151, -16.180586914582452, 789.40251572327043)}

log_folder = '/tmp/log'
run_file = 'runs.png'
data_location = None

graphics=False


current_position=[0,0,0] # x, y, yaw
# current_position=[1322,1069,0] # x, y, yaw

minimum_distance_gazebo=0.7
previous_position_gazebo=[] # x,y, yaw
new_position_flag=False

# Create a Rectangle patch as Nx2
# current_pose = patches.Rectangle((50,100),40,30,linewidth=1,edgecolor='r',facecolor='None')
# origin_arrow_map = np.asarray([[5,0],[0,-2],[0,-1],[-2.,-1],[-2.,1],[0,1],[0,2]])
# origin_arrow_map = np.asarray([[ 0. ,  0. ], [ 2. ,  0. ], [ 2. ,  0.5], [ 3. ,  0. ], [ 2. , -0.5], [ 2. ,  0. ]])
origin_arrow_map = np.asarray([[0.,0.],[7.,0.],[7.,1.5],[9.,0.],[7.,-1.5],[7.,0.]])
# Use recovery icon in case recovery is on...
if rospy.has_param('recovery'):
  evaluate=False
  if rospy.has_param('evaluate'): evaluate=rospy.get_param('evaluate')
  if rospy.get_param('recovery') and not evaluate:
    a=30*np.pi/180
    origin_arrow_left=np.transpose(np.matmul(np.asarray([[np.cos(a), -np.sin(a)],[np.sin(a), np.cos(a)]]), np.transpose(origin_arrow_map)))
    origin_arrow_right=np.transpose(np.matmul(np.asarray([[np.cos(a), np.sin(a)],[-np.sin(a), np.cos(a)]]), np.transpose(origin_arrow_map)))
    origin_arrow_map=np.concatenate([origin_arrow_map, origin_arrow_left, origin_arrow_right], axis=0)
    print("ground_truth_listener: using recovery arrow. {0}".format(origin_arrow_map.shape))

transformed_arrow = origin_arrow_map[:]

# rotation_gazebo_map = np.asarray([[-1,0,1],[0,1,0],[0,0,-1]])
rotation_gazebo_map = np.asarray([[-1,0],[0,1]])

# colors fro plotting
current_color=[0,0,0]
color_transition='rb' #One of ['rb','rg','bg','br','gb','gr']
index_translation={'r':0, 'b':1, 'g':2}

# Create figure and axes
fig,ax = plt.subplots(1,figsize=(30,30))
ax.set_title('Position Display')
current_image = np.zeros((1069,1322))
implot=ax.imshow(current_image,animated=True)

  

def update_color():
  """Update color according to following fields:
  current_color
  color_transition
  """
  global current_color, color_transition
  current_color[index_translation[color_transition[0]]]-=1/20.
  current_color[index_translation[color_transition[1]]]+=1/20.
  if max(current_color) >= 1:
    color_transition=color_transition[1]+color_transition[0]
    current_color[index_translation[color_transition[0]]]-=1/20.
    current_color[index_translation[color_transition[1]]]+=1/20.
  return current_color
  # return (1,1,1)

def animate(*args):
  global new_position_flag
  # implot.set_array(current_image)
  # plt.plot(current_position[0],current_position[1], 'bo')
  # Add the patch to the Axes
  if new_position_flag and not finished:
    ax.add_patch(patches.Polygon(transformed_arrow,linewidth=1,edgecolor=update_color(),facecolor='None'))
    new_position_flag=False

  return implot,


def transform(x,y):
  if len(transformations[img_type]) == 4:
    a,b,c,d=transformations[img_type]
    return (a*x+b, c*y+d)
  else:
    a,b,c,d,e,f=transformations[img_type]
    return (a*x+b*y+c, d*x+e*y+f)

def ready_cb(data):
  global ready, finished, run_file, last_position, img, color_transition, current_color
  if not ready: 
    # reset color variables
    color_transition=np.random.choice(['rb','rg','bg','br','gb','gr'])
    current_color=[1,1,1]
    current_color[index_translation[color_transition[1]]]=0

    ready = True
    finished = False
    evaluation_tag=''
    if rospy.has_param('evaluate'):
      evaluation_tag='evaluate' if rospy.get_param('evaluate') else 'train'
    
    run_file = 'gt_{0:05d}_{1}{2}.png'.format(len([f for f in os.listdir(log_folder) if 'gt' in f and f.endswith('.png') ]), img_type, evaluation_tag)

    last_position = []

def finished_cb(data):
  global ready, finished, fig
  if not finished:
    ready = False
    finished = True
    
    #write last pose to reuse if necessary
    with open(log_folder+'/../last_position.txt','w') as f: 
      f.write("{0[0]}, {0[1]}, {0[2]}, {0[3]}\n".format(last_position))
    
    time.sleep(0.5)
    # if not graphics:
    fig.savefig(log_folder+'/'+run_file)

    time.sleep(0.1)
    clear_fig()

def clear_fig():
  """Clear figure and restart collection of position
  """
  global fig, ax, implot
  plt.cla()
  ax.set_title('Position Display')
  implot=ax.imshow(current_image,animated=True)

def gt_callback(data):
  global last_position, current_position, transformed_arrow,previous_position_gazebo, new_position_flag
  if not ready: return 
  
  x,y=transform(data.pose.pose.position.x,data.pose.pose.position.y)
  
  if len(previous_position_gazebo)==0: 
    previous_position_gazebo=[data.pose.pose.position.x,data.pose.pose.position.y]

  # draw traingle only if distance is far enough
  if np.sqrt((data.pose.pose.position.x-previous_position_gazebo[0])**2+(data.pose.pose.position.y-previous_position_gazebo[1])**2) > minimum_distance_gazebo:
    previous_position_gazebo=[data.pose.pose.position.x,data.pose.pose.position.y]

    # obtain drone_to_gazebo_rotation
    quaternion = (data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

    drone_gazebo_orientation = np.asarray([[np.cos(yaw), -np.sin(yaw)],[np.sin(yaw), np.cos(yaw)]])
    # combine with rotation gazebo_map to get orientation from drone to map
    # combine with translation
    transformation_map_to_drone = np.zeros((3,3))
    transformation_map_to_drone[2,2] = 1
    transformation_map_to_drone[0:2,2] = [x,y]
    transformation_map_to_drone[0:2,0:2] = inv(np.matmul(rotation_gazebo_map, drone_gazebo_orientation))
    # transformation_map_to_drone[0:2,0:2] = np.identity(2)
    # apply transformation to points in arrow
    transformed_arrow=np.transpose(np.matmul(transformation_map_to_drone,np.concatenate([np.transpose(origin_arrow_map),np.ones((1,origin_arrow_map.shape[0]))],axis=0)))
    transformed_arrow=transformed_arrow[:,:2]
    new_position_flag=True

    last_position=[data.pose.pose.position.x,
                  data.pose.pose.position.y,
                  data.pose.pose.position.z,
                  yaw]


    # if not graphics:
    animate()
      
    # # with open(log_folder+'/pos.txt','a') as f: 
    with open(log_folder+'/'+run_file.replace('png','txt'),'a') as f: 
      f.write("{0}, {1}, {2}\n".format(x,y,yaw))
    

  # # if graphics:
  # #   current_position=[x,y,0]

  # # else: positions.append(transform(-data.pose.pose.position.y, data.pose.pose.position.x))
  # if len(positions) > 10**4: positions.pop(0) #avoid memory leakage

def draw_positions(file_name):
  # DEPRECATED
  # f=open('/home/klaas/gt.txt', 'w')
  # for pos in positions:
  #   f.write('{0}\n'.format(str(pos)))
  # f.close()
  print(len(positions))
  fig = plt.figure(figsize=(10, 10))
  imgplot = plt.imshow(img)
  x = [p[0] for p in positions]
  y = [p[1] for p in positions]
  plt.plot(x,y, 'v',ms=1)
  plt.axis('off')
  # plt.show()
  print("[gt_listener]: print run to {}".format(file_name))
  plt.savefig(file_name, bbox_inches='tight')
  


def cleanup():
  """Get rid of the animation on shutdown"""
  plt.close(fig)
  plt.close()


if __name__=="__main__":
  rospy.init_node('gt_listener', anonymous=True)

  simulation_supervised_demo_dir=subprocess.check_output(shlex.split("rospack find simulation_supervised_demo"))[:-1]

  if rospy.has_param('world_name') :
    world = rospy.get_param('world_name')
    if world in transformations.keys(): 
      img_type=world
    img_file = simulation_supervised_demo_dir+'/worlds/'+world+'.png'
    print("[gt_listener]: world: {}".format(img_type))

  if rospy.has_param('background') and rospy.get_param('background') != '':
    if len(rospy.get_param('background')) != 0:
      img_file=rospy.get_param('background')
  try:
    current_image=mpimg.imread(img_file)
  except Exception as e:
    print('[gt_listener]: failed to load background image: '+img_file+'. '+str(e))
  else:
    implot=ax.imshow(current_image,animated=True)
    print("[gt_listener]: image shape: {}".format(current_image.shape))
    # print("[gt_listener]: bg shape: {}".format(current_image.shape))

  log_folder = '/tmp/log'
  if rospy.has_param('log_folder'):
    log_folder=rospy.get_param('log_folder')
  log_folder=log_folder+'/runs'
  if not os.path.isdir(log_folder): 
    os.makedirs(log_folder)
  
  save_images = False
  if rospy.has_param('save_images'):
    save_images = rospy.get_param('save_images')
  
  # in case of dataset creation --> save run also in data_location
  if save_images and rospy.get_param('data_location'):
    data_location = rospy.get_param('data_location')
    
  ready_sub = rospy.Subscriber('/gt_listener_start', Empty, ready_cb)
  finished_sub = rospy.Subscriber('/gt_listener_stop', Empty, finished_cb)
  
  if rospy.has_param('gt_info'):
    rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_callback)
    if rospy.get_param('gt_info')=='/odom':
      turtle = True #switch x and y
      print('[gt_listener]: turtle=on.')
  else:
    print('[gt_listener]: no gt_topic found so switch off.')
    exit(1)

  # if rospy.has_param('graphics'):
  #   if rospy.get_param('graphics'):
  #     graphics=True
  #     print("[ground_truth_listener]: showing graphics.")
  #     anim=animation.FuncAnimation(fig,animate)
  #     plt.show()
  # rospy.on_shutdown(cleanup)

  r = rospy.Rate(30) # 10hz
  while not rospy.is_shutdown():  
      r.sleep()  
