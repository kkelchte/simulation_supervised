#!/usr/bin/env python
import rospy
import numpy as np
import os

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from std_msgs.msg import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry

#
# Listen to current ground truth positions 
# in order to create evaluation plots
#
turtle=False
size = (200,200)
img = np.zeros(size)
img_type = "unknown"
state = ''
current_pos = []
ready = False
finished = True
transformations={'unknown':(1,1,1,1),
		'forest':(6.5, 438.0, -6.6, 418.0),
		'canyon':(15.5, 424.0, -10.6, 809),
		'sandbox':(38.676923076923075, 438.0, -39.876923076923077, 418.0),
		'esat_v1':(16.947783251231524, 63.724137931034484, -16.548448275862071, 590.18620689655177),
		'esat_v2':(17.22058089465456, 1065.4257425742574, -17.138477795147935, 843.90099009900985),
		'forest_real':(17.2170726,785,-17.07010944,487)}
		# 'esat_v2':(16.321360645256139, 1006.9433962264151, -16.180586914582452, 789.40251572327043)}

log_folder = '/tmp/log'
run_file = 'runs.png'

def transform(x,y):
	a,b,c,d=transformations[img_type]
	return (a*x+b, c*y+d)


def draw_positions():
	# f=open('/home/klaas/gt.txt', 'w')
	# for pos in current_pos:
	# 	f.write('{0}\n'.format(str(pos)))
	# f.close()
	fig = plt.figure(figsize=(10, 10))
	imgplot = plt.imshow(img)
	x = [p[0] for p in current_pos]
	y = [p[1] for p in current_pos]
	plt.plot(x,y, 'v',ms=1)
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
	global current_pos
	if not ready: return
	if not turtle: current_pos.append(transform(data.pose.pose.position.x,data.pose.pose.position.y))
	else:	current_pos.append(transform(-data.pose.pose.position.y, data.pose.pose.position.x))


if __name__=="__main__":
  	rospy.init_node('gt_listener', anonymous=True)
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
			print('[GT_LISTENER]: failed to load background image: '+img_file+'. '+str(e))
  	save_images = False
	if rospy.has_param('save_images'):
		save_images = rospy.get_param('save_images')
		print 'save_images ',save_images
	if rospy.has_param('log_folder'):
		log_folder=rospy.get_param('log_folder')
		if save_images and rospy.get_param('saving_location'): # in case of dataset creation
			log_folder = rospy.get_param('saving_location')
		else:
			log_folder=log_folder+'/runs'
			if not os.path.isdir(log_folder):
		 		os.makedirs(log_folder)
	 		run_file = 'gt_{0:05d}_{1}.png'.format(len([f for f in os.listdir(log_folder) if 'gt' in f ]), img_type)
	 		print log_folder
	 		print 'RUNFILE: ',run_file
	else:
		log_folder = '/tmp/log'
	if rospy.has_param('ready'): 
		ready_sub = rospy.Subscriber(rospy.get_param('ready'), Empty, ready_cb)
	
	if rospy.has_param('finished'):
		finished_sub = rospy.Subscriber(rospy.get_param('finished'), Empty, finished_cb)
	
	if rospy.has_param('gt_info'):
		rospy.Subscriber(rospy.get_param('gt_info'), Odometry, gt_callback)
		if rospy.get_param('gt_info')=='/odom':
			turtle = True #switch x and y
			print('[gt_listener]: turtle=on.')
	else:
		exit


	rospy.spin()
	