#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard
t --> take_off
l --> land
f --> flat_trim
e --> emergency
g --> go
o --> overtake
x --> turn left
c --> straight
v --> turn right
CTRL-C to quit
"""

keyBindings = {
  't':'takeoff',
  'l':'land',
  'f':'flattrim',
  'e':'emergency',
  'g':'go',
  'o':'overtake'
}

keyPubs = {
  'takeoff':None,
  'land':None,
  'flattrim':None,
  'emergency':None,
  'go':None,
  'overtake':None
}

action_bound=0.7
speed=0.6
turnspeed=0.6
moveBindings = {
    'x':(turnspeed,0,0,0,0,action_bound),
    'c':(speed,0,0,0,0,0),
    'v':(turnspeed,0,0,0,0,-action_bound)}

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key


def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  rospy.init_node('teleop_twist_keyboard')
  
  # add default publishers /go and /overtake
  keyPubs['go']=rospy.Publisher('/go', Empty, queue_size =1)
  keyPubs['overtake']=rospy.Publisher('/overtake', Empty, queue_size =1)

  # if rospy.hasparam('speed')
  
  vel_pub = rospy.Publisher('key_vel', Twist, queue_size=1)
  for k in keyPubs.keys():
    if rospy.has_param(k):
      keyPubs[k] = rospy.Publisher(rospy.get_param(k), Empty, queue_size = 1)


  try:
    print(msg)
    while(1):
      key = getKey()
      if key in keyBindings.keys():
        print(keyBindings[key])
        if keyBindings[key] in keyPubs.keys():
          keyPubs[keyBindings[key]].publish()
        else:
          print("[teleop_twist_keyboard]: could not find {} publisher".format(keyBindings[key]))
      if key in moveBindings.keys():
        twist=Twist()
        twist.linear.x=moveBindings[key][0]
        twist.linear.y=moveBindings[key][1]
        twist.linear.z=moveBindings[key][2]
        twist.angular.x=moveBindings[key][3]
        twist.angular.y=moveBindings[key][4]
        twist.angular.z=moveBindings[key][5]
        vel_pub.publish(twist)
      if (key == '\x03'):
        break
  except Exception as e:
    print(e)
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

