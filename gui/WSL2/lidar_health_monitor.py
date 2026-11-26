#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""ROS1/Melodic Python2, READ-ONLY Velodyne point-cloud freshness monitor.

Uses rospy.AnyMsg so no large PointCloud2 deserialization and never sends
motion/control commands. The GUI decides whether to restart its owned driver.
"""
from __future__ import print_function
import argparse
import json
import os
import sys
import time


def atomic_snapshot(path, payload):
    tmp = path + '.tmp'
    with open(tmp, 'w') as stream:
        json.dump(payload, stream, sort_keys=True)
        stream.flush()
    os.rename(tmp, path)


def run(path):
    import rospy
    rospy.init_node('hunter_lidar_health_monitor', anonymous=True, disable_signals=False)
    state = {'count': 0, 'last_points_wall': 0.0}

    def callback(_message):
        # This callback does not deserialize the heavy point cloud.
        state['last_points_wall'] = time.time()
        state['count'] += 1

    # Large socket buffer; queue only the freshest message for low overhead.
    rospy.Subscriber('/velodyne_points', rospy.AnyMsg, callback,
                     queue_size=1, buff_size=16 * 1024 * 1024)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        now = time.time()
        atomic_snapshot(path, {
            'updated_wall': now,
            'last_points_wall': state['last_points_wall'],
            'count': state['count'],
        })
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--state-file', required=True)
    args = parser.parse_args()
    try:
        run(args.state_file)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print('LiDAR health monitor error: {}'.format(exc), file=sys.stderr)
        raise
