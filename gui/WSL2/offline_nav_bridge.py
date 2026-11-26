#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""ROS Melodic (Python 2): bridge a recorded NDT trajectory to offline nav.

No motor/CAN topics are published. This is only a demonstration of the
waypoint -> Pure Pursuit -> Twist command-generation chain. It is not an
obstacle-avoidance planner and must not be used to drive a real vehicle.
"""

from __future__ import print_function

import argparse
from route_safety import load_route, validate

import copy
import math

import rospy
from autoware_msgs.msg import Lane, LaneArray, Waypoint
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
from nav_waypoint_progress import (ProgressTracker, read_progress,
                                   write_progress, distance_to_remaining)


class OfflineNavBridge(object):
    # Autoware.AI 1.14 Pure Pursuit resets its received-waypoint flag after
    # each calculation, so a one-time latched publication is not sufficient.
    ROUTE_PUBLISH_HZ = 10.0

    def __init__(self, csv_path=None, cruise_mps=None):
        self.pose_pub = rospy.Publisher(
            "/offline_nav/current_pose", PoseStamped, queue_size=10)
        self.velocity_pub = rospy.Publisher(
            "/offline_nav/current_velocity", TwistStamped, queue_size=10)
        self.lane_pub = rospy.Publisher(
            "/offline_nav/final_waypoints", Lane, queue_size=1, latch=True)
        self.last_position = None
        self.last_stamp = None
        self.latest_velocity = None
        self.latest_velocity_walltime = None
        self.current_route = None
        self.current_pose_xy = None
        self.cruise_mps = cruise_mps
        self.csv_path = csv_path
        self.csv_points = None
        self.progress = None
        self.route_following = False
        self.route_active_walltime = None
        self.was_following = False
        self.visual_last = 0.0
        self.remain_pub = rospy.Publisher('/hunter_nav/remaining_route', Path, queue_size=1, latch=True)
        self.wp_marker_pub = rospy.Publisher('/hunter_nav/waypoint_markers', MarkerArray, queue_size=1, latch=True)
        self.progress_pub = rospy.Publisher('/hunter_nav/progress_index', Int32, queue_size=1, latch=True)
        self.progress_status_pub = rospy.Publisher('/hunter_nav/progress_status', String, queue_size=1, latch=True)
        rospy.Subscriber('/hunter_nav/route_active', Bool, self.on_route_active, queue_size=2)
        rospy.Subscriber('/hunter_nav/safety_state', String, self.on_safety_state, queue_size=2)
        self.route_path_pub = rospy.Publisher('/hunter_nav/route', Path, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('/hunter_nav/goal', PoseStamped, queue_size=1, latch=True)
        if csv_path:
            self.load_csv_direct(csv_path)
        else:
            rospy.Subscriber("/based/lane_waypoints_raw", LaneArray, self.on_lanes,
                             queue_size=1)
        rospy.Subscriber("/ndt_pose", PoseStamped, self.on_pose, queue_size=10)
        rospy.Subscriber("/estimate_twist", TwistStamped, self.on_twist,
                         queue_size=10)
        # ROS-time timer: pauses together with rosbag --clock --pause.
        self.route_timer = rospy.Timer(
            rospy.Duration(1.0 / self.ROUTE_PUBLISH_HZ),
            self.publish_route_periodically)
        rospy.logwarn("Offline bridge: only /offline_nav/* outputs; NO vehicle control")

    def on_lanes(self, msg):
        if not msg.lanes:
            rospy.logwarn("waypoint_loader published an empty LaneArray")
            return
        lane = msg.lanes[0]
        if len(lane.waypoints) < 3:
            rospy.logwarn("Need >=3 waypoints for offline Pure Pursuit")
            return
        self.current_route = copy.deepcopy(lane)
        self.publish_path()
        # Publish immediately as well; the latched copy serves late joiners.
        self.lane_pub.publish(self.current_route)
        rospy.loginfo("Offline route ready: %d waypoints", len(lane.waypoints))


    def load_csv_direct(self, csv_path):
        """Bypass fragile waypoint_loader parameter wiring; subscribe to our own CSV."""
        points = load_route(csv_path)
        result = validate(points)
        lane = Lane()
        lane.header.frame_id = 'map'
        for p in points:
            wp = Waypoint()
            wp.pose.header.frame_id = 'map'
            wp.pose.pose.position.x = p['x']
            wp.pose.pose.position.y = p['y']
            wp.pose.pose.position.z = p['z']
            wp.pose.pose.orientation.z = math.sin(p['yaw'] * 0.5)
            wp.pose.pose.orientation.w = math.cos(p['yaw'] * 0.5)
            # Autoware's Waypoint message stores m/s. CSV velocity is km/h.
            wp.twist.twist.linear.x = (0.0 if p['velocity']<=0.0 else
                                      self.cruise_mps if self.cruise_mps is not None else
                                      p['velocity'] / 3.6)
            lane.waypoints.append(wp)
        self.current_route = lane
        self.csv_points = points
        self.progress = ProgressTracker(points, read_progress(csv_path, points))
        self.publish_path()
        self.publish_progress_visuals()
        self.lane_pub.publish(lane)
        rospy.logwarn('Validated route direct CSV: %s, %d points %.2fm; no vehicle control',
                      csv_path, result['count'], result['length_m'])

    def publish_path(self):
        if self.current_route is None:
            return
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        for wp in self.current_route.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose = copy.deepcopy(wp.pose.pose)
            path.poses.append(pose)
        self.route_path_pub.publish(path)
        if path.poses:
            self.goal_pub.publish(path.poses[-1])

    def on_route_active(self, msg):
        import time
        self.route_following = bool(msg.data)
        self.route_active_walltime = time.time()
        if self.route_following:
            self.was_following = True

    def on_safety_state(self, msg):
        if not (self.progress is not None and self.was_following and
                msg.data.startswith('GOAL:') and self.current_pose_xy is not None):
            return
        last = self.csv_points[-1]
        if math.hypot(self.current_pose_xy[0] - last['x'],
                      self.current_pose_xy[1] - last['y']) > 0.70:
            return
        self.route_following = False
        self.progress.next_index = 0
        self.progress.completed = True
        try:
            write_progress(self.csv_path, self.csv_points, 0)
        except (IOError, OSError) as exc:
            rospy.logerr_throttle(5.0, 'Failed to save goal completion: %s', exc)
        self.publish_progress_visuals()
        self.was_following = False

    def publish_progress_visuals(self):
        if self.current_route is None:
            return
        import time
        self.visual_last = time.time()
        count = len(self.current_route.waypoints)
        next_idx = self.progress.next_index if self.progress is not None else 0
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        # Remaining route includes the last passed point as a continuous anchor.
        first = max(0, next_idx - 1)
        for wp in self.current_route.waypoints[first:]:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose = copy.deepcopy(wp.pose.pose)
            # The test site is planar: render route progress on map XY, rather
            # than drawing the recorded NDT Z drift as floating waypoint arcs.
            # Only this RViz display copy is flattened; CSV and control stay unchanged.
            pose.pose.position.z = 0.03
            path.poses.append(pose)
        self.remain_pub.publish(path)
        markers = MarkerArray()
        clear = Marker()
        clear.header = path.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, wp in enumerate(self.current_route.waypoints):
            if index < next_idx:
                continue
            point = wp.pose.pose.position
            dot = Marker()
            dot.header = path.header
            dot.ns = 'remaining_waypoints'
            dot.id = index * 2
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = point.x, point.y, 0.15
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.20 if index != next_idx else 0.32
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = (1.0, 0.65, 0.0, 1.0) if index == next_idx else (0.12, 0.95, 0.33, 1.0)
            markers.markers.append(dot)
            text = Marker()
            text.header = path.header
            text.ns = 'waypoint_xy_labels'
            text.id = index * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = point.x, point.y, 0.55
            text.pose.orientation.w = 1.0
            text.scale.z = 0.24
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 0.95
            text.text = 'WP{:02d} ({:.2f}, {:.2f})'.format(index + 1, point.x, point.y)
            markers.markers.append(text)
        self.wp_marker_pub.publish(markers)
        self.progress_pub.publish(Int32(next_idx))
        self.progress_status_pub.publish(String('WP {}/{}{}'.format(
            min(next_idx + 1, count), count, ' (COMPLETE, RESET)' if self.progress and self.progress.completed else '')))

    def publish_route_periodically(self, _event):
        if self.current_route is not None:
            self.lane_pub.publish(self.make_local_lane())
            import time
            if time.time() - self.visual_last > 0.65:
                self.publish_progress_visuals()

    def make_local_lane(self):
        """Present a forward-moving route, as Autoware's final_waypoints does.

        Pure Pursuit uses the *first* waypoint's velocity as its command. A
        permanently republished full CSV would keep commanding the initial
        zero-speed waypoint even after the recorded vehicle moves past it.
        """
        lane = copy.deepcopy(self.current_route)
        if self.current_pose_xy is None:
            return lane

        px, py = self.current_pose_xy
        waypoints = lane.waypoints
        nearest = min(
            range(len(waypoints)),
            key=lambda index: (
                (waypoints[index].pose.pose.position.x - px) ** 2 +
                (waypoints[index].pose.pose.position.y - py) ** 2))
        # Never rewind to visited points when the vehicle is manually moved.
        next_idx = self.progress.next_index if self.progress is not None else 0
        first = min(max(0, next_idx - 1), len(waypoints) - 3)
        lane.waypoints = waypoints[first:]

        # The final point in the recorded CSV is a stop point. If the closest
        # point is among the final two, command zero speed instead of reusing
        # the third-to-last point's nonzero speed.
        if (self.progress is not None and self.progress.completed) or nearest >= len(waypoints) - 2:
            lane.waypoints[0].twist.twist.linear.x = 0.0
        return lane

    def on_twist(self, msg):
        self.latest_velocity = msg
        self.latest_velocity_walltime = rospy.get_time()

    def on_pose(self, pose):
        self.current_pose_xy = (pose.pose.position.x, pose.pose.position.y)
        if self.progress is not None:
            import time
            active_recent = (self.route_following and self.route_active_walltime is not None and
                             time.time()-self.route_active_walltime < 0.45)
            # Use actual orientation to disambiguate nearby opposite-direction
            # U-turn legs, while only advancing when the supervisor is ACTIVE.
            q=pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            changed = self.progress.update(pose.pose.position.x, pose.pose.position.y,
                                           active_recent, yaw=yaw)
            if changed:
                rospy.loginfo('Waypoint progress: next WP %d/%d (local spacing reach %.2fm); active=%s',
                              self.progress.next_index+1,len(self.csv_points),
                              self.progress.local_reach(self.progress.next_index),active_recent)
                try:
                    write_progress(self.csv_path, self.csv_points, self.progress.next_index)
                except (IOError, OSError) as exc:
                    rospy.logerr_throttle(5.0, 'Failed to save waypoint progress: %s', exc)
                self.publish_progress_visuals()
                if self.progress.completed:
                    rospy.logwarn('Route complete: waypoint markers reset to full path; STOP required before next run')
            # After goal completion, the operator may manually return to WP01.
            # This re-enables the prepared CSV for the NEXT GUI START; it never
            # emits a motor command or bypasses the supervisor/starting check.
            if self.progress.completed and not active_recent:
                start=self.csv_points[0]
                if math.hypot(pose.pose.position.x-start['x'],
                              pose.pose.position.y-start['y']) <= 0.45:
                    self.progress.completed=False
                    self.publish_progress_visuals()
        self.pose_pub.publish(pose)
        velocity = TwistStamped()
        velocity.header = pose.header
        # Prefer Autoware's NDT velocity estimate if it is recent. Fallback to
        # finite differences to keep a meaningful input when estimate_twist is
        # unavailable in a particular build. No actuation happens here.
        if (self.latest_velocity is not None
                and self.latest_velocity_walltime is not None
                and abs(rospy.get_time() - self.latest_velocity_walltime) < 0.8):
            velocity.twist = copy.deepcopy(self.latest_velocity.twist)
        elif self.last_position is not None and self.last_stamp is not None:
            dt = (pose.header.stamp - self.last_stamp).to_sec()
            if 0.02 < dt < 2.0:
                dx = pose.pose.position.x - self.last_position[0]
                dy = pose.pose.position.y - self.last_position[1]
                if math.hypot(dx, dy) < 1.5:
                    velocity.twist.linear.x = min(5.0, math.hypot(dx, dy) / dt)
        self.velocity_pub.publish(velocity)
        self.last_position = (pose.pose.position.x, pose.pose.position.y)
        self.last_stamp = pose.header.stamp


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', default=None)
    parser.add_argument('--cruise-mps',type=float,default=None)
    options = parser.parse_args(rospy.myargv()[1:])
    if options.cruise_mps is not None and not 0.05<=options.cruise_mps<=5.00:
        parser.error('--cruise-mps out of [0.05,5.00]')
    rospy.init_node('offline_nav_bridge', anonymous=False)
    OfflineNavBridge(csv_path=options.csv,cruise_mps=options.cruise_mps)
    rospy.spin()
