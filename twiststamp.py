#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS1 Melodic: bridge Autoware /twist_cmd to the robot's /cmd_vel.

Input:  geometry_msgs/TwistStamped (default: /twist_cmd)
Output: geometry_msgs/Twist        (default: /cmd_vel)

Only linear.x and angular.z are forwarded, as in the original twiststamp.
The watchdog publishes a zero Twist if upstream commands stop arriving.
This is a command bridge, NOT a CAN or Hunter chassis driver.

Parameters (ROS private namespace):
    ~input_topic       (string, default: /twist_cmd)
    ~output_topic      (string, default: /cmd_vel)
    ~cmd_timeout_sec   (float,  default: 1.0)
    ~watchdog_rate_hz  (float,  default: 10.0)

Compatible with the Python 2.7 runtime commonly used by ROS Melodic.
"""

from __future__ import print_function

import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist, TwistStamped


def is_finite(value):
    """Python 2.7-compatible check for NaN and infinity."""
    return not (math.isnan(value) or math.isinf(value))


class TwistStampedToTwist(object):
    def __init__(self):
        rospy.init_node('twiststamped_to_twist')

        self.input_topic = rospy.get_param('~input_topic', '/twist_cmd')
        self.output_topic = rospy.get_param('~output_topic', '/cmd_vel')
        self.timeout_sec = float(rospy.get_param('~cmd_timeout_sec', 1.0))
        self.watchdog_rate = float(rospy.get_param('~watchdog_rate_hz', 10.0))

        if not is_finite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise ValueError('~cmd_timeout_sec must be positive and finite')
        if not is_finite(self.watchdog_rate) or self.watchdog_rate <= 0.0:
            raise ValueError('~watchdog_rate_hz must be positive and finite')
        if self.input_topic == self.output_topic:
            raise ValueError('input and output topics must be different')

        # Use wall-clock time for the watchdog. This avoids relying on /clock
        # (which is primarily intended for rosbag/simulation playback).
        self._clock = getattr(time, 'monotonic', time.time)
        self._lock = threading.Lock()
        self._last_command_time = None
        self._has_received_command = False
        self._timeout_reported = False

        self.cmd_vel_pub = rospy.Publisher(
            self.output_topic, Twist, queue_size=5
        )
        rospy.on_shutdown(self.shutdown)

        self.command_sub = rospy.Subscriber(
            self.input_topic, TwistStamped, self.on_command, queue_size=5
        )
        self.watchdog = rospy.Timer(
            rospy.Duration(1.0 / self.watchdog_rate), self.on_watchdog
        )

        rospy.loginfo(
            'Twist bridge ready: %s (TwistStamped) -> %s (Twist); '
            'timeout=%.2fs',
            self.input_topic, self.output_topic, self.timeout_sec
        )

    def on_command(self, stamped):
        linear_x = stamped.twist.linear.x
        angular_z = stamped.twist.angular.z

        with self._lock:
            if not (is_finite(linear_x) and is_finite(angular_z)):
                # Invalid control data must never be forwarded to the chassis.
                self._last_command_time = None
                self._has_received_command = True
                self.cmd_vel_pub.publish(Twist())
                rospy.logwarn('Invalid /twist_cmd value; publishing zero velocity')
                return

            command = Twist()
            command.linear.x = linear_x
            command.angular.z = angular_z

            # Update the watchdog state and publish atomically so an old
            # timeout cannot overwrite a freshly received valid command.
            self._last_command_time = self._clock()
            self._has_received_command = True
            self._timeout_reported = False
            self.cmd_vel_pub.publish(command)

        rospy.logdebug(
            'Forwarded /twist_cmd: linear.x=%.3f, angular.z=%.3f',
            linear_x, angular_z
        )

    def on_watchdog(self, _event):
        with self._lock:
            elapsed = (
                None if self._last_command_time is None
                else self._clock() - self._last_command_time
            )
            timed_out = elapsed is None or elapsed > self.timeout_sec

            if timed_out:
                # Repeated zeros help a downstream /cmd_vel consumer stop
                # even when the upstream planner is no longer publishing.
                self.cmd_vel_pub.publish(Twist())
                if self._has_received_command and not self._timeout_reported:
                    rospy.logwarn(
                        'No valid /twist_cmd for > %.2fs; publishing zero',
                        self.timeout_sec
                    )
                    self._timeout_reported = True

    def shutdown(self):
        rospy.loginfo('Twist bridge shutting down; requesting zero velocity')
        try:
            self.watchdog.shutdown()
        except (AttributeError, RuntimeError):
            pass

        # ROS publication is asynchronous; send a few zeros without the
        # original script's one-second wait before the first stop request.
        for _ in range(3):
            try:
                self.cmd_vel_pub.publish(Twist())
            except rospy.ROSException:
                break
            time.sleep(0.05)


if __name__ == '__main__':
    try:
        TwistStampedToTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
