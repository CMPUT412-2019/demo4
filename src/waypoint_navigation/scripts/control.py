#!/usr/bin/env python

from curses import wrapper

import rospy
from geometry_msgs.msg import Twist


def main(win):
    rospy.init_node('control_node')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    key_mapping = {
        'w': (1, 0),
        's': (-1, 0),
        'a': (0, 1),
        'd': (0, -1),
        'x': (0, 0),
    }

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key = win.getkey()
        if key in key_mapping:
            t = Twist()
            t.linear.x, t.angular.z = key_mapping[key]
            publisher.publish(t)
        elif key == 'q':
            break


wrapper(main)
