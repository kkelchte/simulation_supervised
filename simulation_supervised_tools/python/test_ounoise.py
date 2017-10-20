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
from ou_noise import OUNoise

x = 0
img = np.zeros((200,800))
preval = 0

def show():
	global x, preval, img
	val = noise.noise()
	cv2.line(img, (x, 10*preval+100),(x+1, 10*val+100),(255,255,255),5,8,0) 
	cv2.imshow('Noise Python',img)
	cv2.waitKey(2)
	preval = val
	x = x+5
	if x > 800 : 
		x=0
		preval = 0

if __name__=="__main__":
	noise = OUNoise(1, 0, 0.15, 1)

	rospy.init_node('ounoise', anonymous=True)
	r = rospy.Rate(5) # 10hz
	while not rospy.is_shutdown():
		show()
		r.sleep()