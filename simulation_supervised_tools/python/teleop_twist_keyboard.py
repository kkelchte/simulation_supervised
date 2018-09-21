#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard
t --> take_off
l --> land
f --> flat_trim
e --> emergency
g --> go
o --> overtake
x --> turn left TODO
c --> straight TODO
v --> turn right TODO
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

moveBindings = {
    'i':(1,0,0,0),
    'o':(1,0,0,-1),
    'j':(0,0,0,1),
    'l':(0,0,0,-1),
    'u':(1,0,0,1),
    ',':(-1,0,0,0),
    '.':(-1,0,0,1),
    'm':(-1,0,0,-1),
    'O':(1,-1,0,0),
    'I':(1,0,0,0),
    'J':(0,1,0,0),
    'L':(0,-1,0,0),
    'U':(1,1,0,0),
    '<':(-1,0,0,0),
    '>':(-1,-1,0,0),
    'M':(-1,1,0,0),
    't':(0,0,1,0),
    'b':(0,0,-1,0),
         }

speedBindings={
    'q':(1.1,1.1),
    'z':(.9,.9),
    'w':(1.1,1),
    'x':(.9,1),
    'e':(1,1.1),
    'c':(1,.9),
        }

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
  
  for k in keyPubs.keys():
    if rospy.has_param(k):
      keyPubs[k] = rospy.Publisher(rospy.get_param(k), Empty, queue_size = 1)

  # import pdb; pdb.set_trace()

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
      # if key in moveBindings.keys():
      #   x = moveBindings[key][0]
      #   y = moveBindings[key][1]
      #   z = moveBindings[key][2]
      #   th = moveBindings[key][3]
      # elif key in speedBindings.keys():
      #   speed = speed * speedBindings[key][0]
      #   turn = turn * speedBindings[key][1]

      #   print(vels(speed,turn))
      #   if (status == 14):
      #     print(msg)
      #   status = (status + 1) % 15
      # else:
      #   x = 0
      #   y = 0
      #   z = 0
      #   th = 0
      if (key == '\x03'):
        break

      # twist = Twist()
      # twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
      # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
      # pub.publish(twist)

  except Exception as e:
    print(e)

  finally:
    # twist = Twist()
    # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    # pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


