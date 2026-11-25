#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""NDT pose -> RViz-only Hunter silhouette and breadcrumb path.

Subscribes only to /current_pose (mapping) OR /ndt_pose (matching).
Publishes visualization_msgs/MarkerArray and nav_msgs/Path in MAP coordinates.
Never publishes /cmd_vel or TF. Dimensions are *schematic*, not a URDF model.
"""
from __future__ import print_function
import argparse
import math
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def rgba(m, r, g, b, a):
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a


def vec(x, y, z):
    p = Point()
    p.x, p.y, p.z = x, y, z
    return p


class PoseVisualizer(object):
    def __init__(self, pose_topic):
        self.pose_topic = pose_topic
        self.markers = rospy.Publisher('/hunter_viz/robot_markers', MarkerArray, queue_size=2)
        self.path_pub = rospy.Publisher('/hunter_viz/path', Path, queue_size=1, latch=True)
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.last_pos = None
        self.count = 0
        rospy.Subscriber(pose_topic, PoseStamped, self.cb, queue_size=1)
        rospy.loginfo('Hunter RViz-only marker: %s -> /hunter_viz/robot_markers and /hunter_viz/path', pose_topic)

    def make_marker(self, pose, marker_id, kind):
        m = Marker()
        m.header = pose.header
        m.ns = 'hunter_ndt_estimate'
        m.id = marker_id
        m.type = kind
        m.action = Marker.ADD
        m.pose = pose.pose
        m.lifetime = rospy.Duration(1.5)
        m.scale.x = m.scale.y = m.scale.z = 1.0
        return m

    def cb(self, msg):
        # NDT poses should be in map. Do not silently relabel an unrelated frame.
        if msg.header.frame_id.lstrip('/') != 'map':
            rospy.logwarn_throttle(5.0, 'Expected NDT PoseStamped frame map, got %s', msg.header.frame_id)
            return
        p = msg.pose.position
        q = msg.pose.orientation
        if not all((not math.isnan(x) and not math.isinf(x)) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            return
        if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 0.1:
            return
        out = MarkerArray()

        body = self.make_marker(msg, 0, Marker.CUBE)
        body.pose.position.z += 0.19
        body.scale.x, body.scale.y, body.scale.z = 1.35, 0.82, 0.30
        rgba(body, 0.18, 0.73, 0.94, 0.72)
        out.markers.append(body)

        nose = self.make_marker(msg, 1, Marker.ARROW)
        nose.scale.x, nose.scale.y, nose.scale.z = 0.08, 0.16, 0.19
        nose.points = [vec(0, 0, 0.32), vec(1.25, 0, 0.32)]
        rgba(nose, 1.0, 0.62, 0.08, 1.0)
        out.markers.append(nose)

        outline = self.make_marker(msg, 2, Marker.LINE_STRIP)
        outline.scale.x = 0.045
        outline.points = [vec(0.72, 0.45, 0.07), vec(-0.72, 0.45, 0.07),
                          vec(-0.72, -0.45, 0.07), vec(0.72, -0.45, 0.07),
                          vec(0.72, 0.45, 0.07)]
        rgba(outline, 0.98, 0.97, 0.26, 1.0)
        out.markers.append(outline)

        name = self.make_marker(msg, 3, Marker.TEXT_VIEW_FACING)
        name.pose.position.z += 0.90
        name.scale.z = 0.36
        name.text = 'HUNTER / NDT'  # schematic, not odometry/GPS ground truth
        rgba(name, 1.0, 1.0, 0.98, 1.0)
        out.markers.append(name)
        self.markers.publish(out)

        yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
        is_new = (self.last_pos is None or
                  math.hypot(p.x-self.last_pos[0], p.y-self.last_pos[1]) >= 0.05 or
                  abs(math.atan2(math.sin(yaw-self.last_pos[2]), math.cos(yaw-self.last_pos[2]))) >= 0.06)
        if is_new:
            point = PoseStamped()
            point.header = msg.header
            point.pose = msg.pose
            self.path.poses.append(point)
            self.path.poses = self.path.poses[-1200:]
            self.last_pos = (p.x, p.y, yaw)
            self.path.header.stamp = msg.header.stamp
            self.path_pub.publish(self.path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pose-topic', required=True, choices=['/current_pose', '/ndt_pose'])
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('hunter_ndt_pose_visualizer')
    PoseVisualizer(args.pose_topic)
    rospy.spin()


if __name__ == '__main__':
    main()
