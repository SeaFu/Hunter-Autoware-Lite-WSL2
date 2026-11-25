#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Operator-deadman publisher: only a GUI-main-thread heartbeat keeps True.

The GUI writes 'H' to stdin every 100ms WHILE the hold button is pressed.
If GUI stalls, closes or stdin breaks, publishing True expires in 0.30s.
This is not a safety-rated emergency stop.
"""
from __future__ import print_function
import os
import select
import sys
import time
import rospy
from std_msgs.msg import Bool


def main():
    rospy.init_node('hunter_nav_operator_gate', anonymous=False)
    pub=rospy.Publisher('/hunter_nav/operator_hold',Bool,queue_size=1)
    stamp=0.0
    rate=rospy.Rate(20)
    fd=sys.stdin.fileno()
    while not rospy.is_shutdown():
        try:
            ready,_,_=select.select([fd], [], [], 0)
            if ready:
                data=os.read(fd,4096)
                if not data:
                    pub.publish(Bool(False))
                    break
                if b'0' in data:
                    stamp=0.0
                elif b'H' in data:
                    stamp=time.time()
        except (OSError,ValueError):
            pub.publish(Bool(False))
            break
        pub.publish(Bool(time.time()-stamp<0.30))
        rate.sleep()
    pub.publish(Bool(False))

if __name__=='__main__': main()
