#!/usr/bin/env python
import rospy
import numpy as np
import os

import copy
import tf

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from std_msgs.msg import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import cv2

#
# Listen to evaluate to plot positions
#
size = (200,200)
img = np.zeros(size)
img_type = "unknown"
state = ''
gt_global_pose, prev_global_pose, gt_positions, prev_global_pose_es, es_positions = [], [], [], [], []

ready = False
finished = True
font = cv2.FONT_HERSHEY_SIMPLEX

yaw=0
transformations={'unknown':(1,1,1,1),
		'forest':(6.5, 438.0, -6.6, 418.0),
		'canyon':(15.5, 424.0, -10.6, 809),
		'sandbox':(38.676923076923075, 438.0, -39.876923076923077, 418.0),
		'esat_v1':(16.947783251231524, 63.724137931034484, -16.548448275862071, 590.18620689655177),
		'esat_v2':(17.22058089465456, 1065.4257425742574, -17.138477795147935, 843.90099009900985),
		'forest_real':(17.2170726,785,-17.07010944,487)}

log_folder = '/tmp/log'
run_file = 'runs.png'

def transform(x,y):
	a,b,c,d=transformations[img_type]
	return (a*x+b, c*y+d)

def show_prediction(dx_es,dy_es,dx_gt,dy_gt):
	img=np.ones((150,150,3))
	xc=int(img.shape[1]/2)
	yc=int(img.shape[0]/2)
	l=500
	cv2.line(img, (xc, yc), (int(xc-l*dx_gt),int(yc-l*dy_gt)),(0,0,0), 3)
	cv2.line(img, (xc, yc), (int(xc-l*dx_es),int(yc-l*dy_es)),(0,255,0), 3)
	cv2.line(img, (2*xc, 2*yc), (int(xc-20),2*yc),(0,0,255), 1)
	cv2.line(img, (2*xc, 2*yc), (2*xc,2*yc-20),(0,0,255), 1)
	cv2.imshow('Odom',img)
	cv2.waitKey(2)

def draw_positions():
	if len(es_positions) == 0:
		print('[SHOW_ODOM] Did NOT write odom predictions.')
		return
	fig = plt.figure(figsize=(10, 10))
	imgplot = plt.imshow(img)
	t_gt_positions = [transform(pos[0], pos[1]) for pos in gt_positions]
	x = [p[0] for p in t_gt_positions]
	y = [p[1] for p in t_gt_positions]
	plt.plot(x,y, 'bo',ms=2)
	t_es_positions = [transform(pos[0], pos[1]) for pos in es_positions]
	x = [p[0] for p in t_es_positions]
	y = [p[1] for p in t_es_positions]
	plt.plot(x,y, 'gs',ms=2)
	plt.axis('off')
	# plt.show()
	plt.savefig(log_folder+'/'+run_file, bbox_inches='tight')
	
def ready_cb(data):
	global ready, finished
	if not ready: 
		ready = True
		finished = False

def finished_cb(data):
	global ready, finished
	if not finished:
		ready = False
		finished = True
		draw_positions()

def gt_callback(data):
	global gt_global_pose
	quaternion = (
	    data.pose.pose.orientation.x,
	    data.pose.pose.orientation.y,
	    data.pose.pose.orientation.z,
	    data.pose.pose.orientation.w)
	position = [data.pose.pose.position.x,
	    data.pose.pose.position.y,
	    data.pose.pose.position.z]
	gt_global_pose = tf.transformations.quaternion_matrix(quaternion) # orientation of current frame relative to global frame
	gt_global_pose[0:3,3]=position
	
def predicted_callback(data):
	global prev_global_pose_es, prev_global_pose, gt_positions, es_positions
	if not ready : return

	# parse estimated global pose from odometry
	pos = [data.data[i] for i in 0,1,2]
	roll,pitch,yaw = data.data[3],data.data[4],data.data[5]
	if len(prev_global_pose_es)==0: #for first time use groundtruth pose in order to map it correctly on background image
		prev_global_pose_es = copy.deepcopy(gt_global_pose)
	cur_rel_pose_es = tf.transformations.euler_matrix(roll,pitch,yaw,axes='rxyz')
	cur_rel_pose_es[0:3,3]=pos
	cur_global_pose_es = tf.transformations.concatenate_matrices(prev_global_pose_es, cur_rel_pose_es)
	prev_global_pose_es = copy.deepcopy(cur_global_pose_es)
	es_positions.append([cur_global_pose_es[0,3],cur_global_pose_es[1,3]])

	# parse ground truth as a sanity check
	pos = [data.data[i+6] for i in 0,1,2]
	roll,pitch,yaw = data.data[6+3],data.data[6+4],data.data[6+5]
	if len(prev_global_pose)==0: #for first time use groundtruth pose in order to map it correctly on background image
		prev_global_pose = copy.deepcopy(gt_global_pose)
	cur_rel_pose = tf.transformations.euler_matrix(roll,pitch,yaw,axes='rxyz')
	cur_rel_pose[0:3,3]=pos
	cur_global_pose = tf.transformations.concatenate_matrices(prev_global_pose, cur_rel_pose)
	prev_global_pose = copy.deepcopy(cur_global_pose)
	gt_positions.append([cur_global_pose[0,3],cur_global_pose[1,3]])
	
	euler_gt = tf.transformations.euler_from_matrix(cur_global_pose, 'rxyz')
	# print 'gt: ',str(euler[2]), str(gt_global_pose[0,3]),str(gt_global_pose[1,3])
	euler_es = tf.transformations.euler_from_matrix(cur_global_pose_es, 'rxyz')
	# print 'estimated: ',str(euler_es[2]), str(cur_global_pose_es[0,3]),str(cur_global_pose_es[1,3])
	# print 'difference: ',str(euler_gt[2]-euler_es[2]),str(cur_global_pose[0,3]-cur_global_pose_es[0,3]),str(cur_global_pose[1,3]-cur_global_pose_es[1,3])
	
	show_prediction(data.data[0],data.data[1],data.data[0+6],data.data[1+6])
	# print("real odom: {0} estimated odom: {1}".format(data.data[4:6], data.data[0:2]))
	# print("current pos: {0} vs estimated pos: {1}".format(gt_positions[-1], es_positions[-1]))


if __name__=="__main__":
  	rospy.init_node('show_odom_prediction', anonymous=True)
	if rospy.has_param('background'):
		img_file=rospy.get_param('background')
		print img_file
		try:
			img=mpimg.imread(img_file)
			img_type = img_file.split("/")[-1].split(".")[0]
			if 'sandbox' in img_type:
				img_type = 'sandbox'
			print("img_type: {}".format(img_type))
		except Exception as e:
			print('[SHOW_ODOM]: failed to load background image: '+img_file+'. '+str(e))
  	if rospy.has_param('log_folder'):
		log_folder=rospy.get_param('log_folder')
		log_folder=log_folder+'/runs'
		if not os.path.isdir(log_folder):
	 		os.mkdir(log_folder)
 		run_file = 'odom_{0:05d}.png'.format(len([f for f in os.listdir(log_folder) if 'odom' in f ]))

 		print log_folder
 		print 'RUNFILE: ',run_file
	else:
		log_folder = '/tmp/log'
	if rospy.has_param('ready'): 
		ready_sub = rospy.Subscriber(rospy.get_param('ready'), Empty, ready_cb)
	if rospy.has_param('finished'): 
  		finished_sub = rospy.Subscriber(rospy.get_param('finished'), Empty, finished_cb)
  	rospy.Subscriber('/ground_truth/state', Odometry, gt_callback)
  	rospy.Subscriber('/odom_prediction', numpy_msg(Floats), predicted_callback, queue_size = 10)

  	rospy.spin()
	