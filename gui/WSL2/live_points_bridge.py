#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Live PointCloud2 routing for Autoware.AI 1.14 (never publishes commands).

Preserves the sensor header/stamp. Runs only for the selected NDT mode.
"""
from __future__ import print_function
import argparse
import rospy
from sensor_msgs.msg import PointCloud2
from pointcloud_stride import thin_cloud


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='/velodyne_points')
    ap.add_argument('--output', required=True, choices=['/points_raw', '/filtered_points'])
    ap.add_argument('--stride', type=int, default=1, choices=[1,2,3,4],
                    help='Every Nth point, matching only; 1 = all points')
    opts = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('hunter_live_points_bridge', anonymous=False)
    pub = rospy.Publisher(opts.output, PointCloud2, queue_size=1)
    def on_cloud(msg):
        try:
            pub.publish(thin_cloud(msg, opts.stride, PointCloud2))
        except Exception as exc:
            # Do not publish a malformed cloud or silently fall back to more CPU.
            rospy.logerr_throttle(5.0, 'PointCloud stride failed: %s', str(exc))
    rospy.Subscriber(opts.source, PointCloud2, on_cloud, queue_size=1, buff_size=12*1024*1024)
    rospy.loginfo('LIVE POINTS: %s -> %s (preserve header), stride=%s', opts.source, opts.output, opts.stride)
    rospy.spin()

if __name__ == '__main__':
    main()
