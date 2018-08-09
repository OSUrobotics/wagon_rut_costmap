#!/usr/bin/env python
import roslib; roslib.load_manifest('intro_nav')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to key_monitor!
---------------------------
h: Return to starting point
r: Randomize waypoint order
CTRL-C to quit
"""

moveBindings = {
		'h':'h',
		'r':'r',
		}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	print key
	return key

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	pub = rospy.Publisher('key_monitor', String, queue_size=10)
	rospy.init_node('teleop_twist_keyboard')
	print msg
	while(1):
		key = getKey()
		if key in moveBindings.keys():
			key_press = key
			pub.publish(key_press)
			# print "in keybindings"
		else:
			if (key == '\x03'):
				break
