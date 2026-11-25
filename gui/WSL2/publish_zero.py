#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Best-effort zero velocity on ROS1 /cmd_vel. Not an emergency stop."""
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('hunter_gui_zero_command', anonymous=True, disable_signals=True)
    pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(0.4)
    hz=rospy.Rate(30)
    for _ in xrange(60):
        if rospy.is_shutdown(): break
        pub.publish(Twist())
        hz.sleep()
