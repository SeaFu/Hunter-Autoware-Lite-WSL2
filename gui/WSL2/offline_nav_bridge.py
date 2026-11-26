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
import threading

import rospy
from autoware_msgs.msg import Lane, LaneArray, Waypoint
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int32, String
from visualization_msgs.msg import Marker, MarkerArray
from nav_trajectory_feasibility import COLORS
from nav_ordered_lookahead import ordered_target, ordered_pursuit
from nav_waypoint_progress import (ProgressTracker, read_progress,
                                   write_progress)


class OfflineNavBridge(object):
    # Autoware.AI 1.14 Pure Pursuit resets its received-waypoint flag after
    # each calculation, so a one-time latched publication is not sufficient.
    ROUTE_PUBLISH_HZ = 10.0

    def __init__(self, csv_path=None, cruise_mps=None, lookahead_ratio=1.17, min_lookahead=2.0):
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
        self.current_yaw = None
        self.cruise_mps = cruise_mps
        self.lookahead_ratio = lookahead_ratio
        self.min_lookahead = min_lookahead
        self.current_speed = 0.0
        self.csv_path = csv_path
        # CSV navigation computes PP directly from the same ordered target
        # as the visualization. The launch starts only the twist_filter.
        # Legacy LaneArray mode still uses the Autoware C++ PP node.
        self.ordered_twist_pub = (rospy.Publisher('/offline_nav/twist_raw',
                                   TwistStamped, queue_size=1) if csv_path else None)
        self.csv_points = None
        self.progress = None
        self.progress_lock = threading.RLock()
        self.last_progress_pose_stamp = None
        self.goal_reset_saved = False
        self.route_following = False
        self.route_active_walltime = None
        self.was_following = False
        self.visual_last = 0.0
        self.remain_pub = rospy.Publisher('/hunter_nav/remaining_route', Path, queue_size=1, latch=True)
        self.wp_marker_pub = rospy.Publisher('/hunter_nav/waypoint_markers', MarkerArray, queue_size=1, latch=True)
        self.feasibility_pub = rospy.Publisher('/hunter_nav/feasibility_markers', MarkerArray, queue_size=1, latch=True)
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
        # Same live config topic as Autoware. Update only when GUI applies
        # parameters (GUI locks these while driving).
        from autoware_config_msgs.msg import ConfigWaypointFollower
        rospy.Subscriber('/offline_nav/config/waypoint_follower', ConfigWaypointFollower,
                         self.on_follow_config, queue_size=1)
        # ROS-time timer: pauses together with rosbag --clock --pause.
        self.route_timer = rospy.Timer(
            rospy.Duration(1.0 / self.ROUTE_PUBLISH_HZ),
            self.publish_route_periodically)
        rospy.logwarn("Offline bridge: only /offline_nav/* outputs; NO vehicle control")
        if self.ordered_twist_pub is not None:
            rospy.logwarn('ORDERED PP DIRECT: arc target -> twist_raw -> Autoware twist_filter -> supervisor; no C++ PP target re-search')

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
        self.lane_pub.publish(self.make_local_lane())
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
        with self.progress_lock:
            import time
            self.route_following = bool(msg.data)
            self.route_active_walltime = time.time()
            if self.route_following:
                self.was_following = True

    def on_safety_state(self, msg):
        with self.progress_lock:
            # Only the supervisor can confirm completion, and only after the
            # tracker has consumed all previous WPs. A nearby goal alone is not
            # permission to reset the route.
            if not (self.progress is not None and
                    msg.data.startswith('GOAL: 最後一個 Waypoint 已依序抵達')):
                return
            if not self.progress.completed and not self.progress.acknowledge_goal():
                return
            if getattr(self, 'goal_reset_saved', False):
                return
            self.route_following = False
            # Even if writing fails, keep completion latched and output zero;
            # retry on the supervisor's repeated GOAL status.
            try:
                write_progress(self.csv_path, self.csv_points, 0)
                self.goal_reset_saved = True
            except (IOError, OSError) as exc:
                rospy.logerr_throttle(5.0, 'Failed to save goal completion: %s', exc)
            self.publish_progress_visuals()
            self.was_following = False
            if self.goal_reset_saved:
                rospy.logwarn('Route complete: disk progress reset; explicit STOP/START required for next run')

    def publish_progress_visuals(self):
        with self.progress_lock:
            if self.current_route is None:
                return
            import time
            self.visual_last = time.time()
            count = len(self.current_route.waypoints)
            complete = bool(self.progress and self.progress.completed)
            next_idx = 0 if complete else (self.progress.next_index if self.progress is not None else 0)
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
            self.publish_saved_feasibility(path.header, next_idx)
            # Keep wire progress terminal even though markers/disk reset to WP01.
            self.progress_pub.publish(Int32(count if complete else next_idx))
            self.progress_status_pub.publish(String('WP {}/{}{}'.format(
                count if complete else min(next_idx + 1, count), count,
                ' (COMPLETE, RESET; STOP/START)' if complete else '')))

    def publish_saved_feasibility(self, header, next_idx):
        """Restore exactly the segment statuses saved in the same CSV.

        The markers are informational and never change obstacle or motor logic.
        Display only uncompleted segments while following the route.
        """
        arr=MarkerArray()
        clear=Marker();clear.header=header;clear.action=Marker.DELETEALL
        arr.markers.append(clear)
        if self.csv_points:
            from geometry_msgs.msg import Point
            for i in range(max(0,next_idx-1),len(self.csv_points)-1):
                a,b=self.csv_points[i:i+2]
                status=a.get('segment_status','PENDING')
                color=COLORS.get(status,COLORS['PENDING'])
                marker=Marker();marker.header=header;marker.ns='csv_feasibility_segments'
                marker.id=i+1;marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD
                marker.pose.orientation.w=1.0;marker.scale.x=.085
                marker.color.r,marker.color.g,marker.color.b,marker.color.a=color+(1.0,)
                for q in (a,b):
                    point=Point();point.x=q['x'];point.y=q['y'];point.z=.12
                    marker.points.append(point)
                arr.markers.append(marker)
                if status in ('WARN','FAIL') and i%2==0:
                    text=Marker();text.header=header;text.ns='csv_segment_reasons';text.id=i+1
                    text.type=Marker.TEXT_VIEW_FACING;text.action=Marker.ADD
                    text.pose.orientation.w=1.0;text.pose.position.x=(a['x']+b['x'])/2.0
                    text.pose.position.y=(a['y']+b['y'])/2.0;text.pose.position.z=.55
                    text.scale.z=.18
                    text.color.r,text.color.g,text.color.b,text.color.a=color+(1.0,)
                    text.text='S{:02d} {}'.format(i+1,a.get('segment_reason','')[:19])
                    arr.markers.append(text)
        self.feasibility_pub.publish(arr)

    def publish_route_periodically(self, _event):
        if self.current_route is not None:
            self.lane_pub.publish(self.make_local_lane())
            import time
            if time.time() - self.visual_last > 0.65:
                self.publish_progress_visuals()

    def on_follow_config(self,msg):
        ratio,minimum=msg.lookahead_ratio,msg.minimum_lookahead_distance
        if .1<=ratio<=20. and .1<=minimum<=30.:
            with self.progress_lock:
                self.lookahead_ratio,self.min_lookahead=ratio,minimum

    @staticmethod
    def interpolated_wp(a,b,t):
        result=copy.deepcopy(b)
        pa,pb,pc=a.pose.pose.position,b.pose.pose.position,result.pose.pose.position
        for axis in ('x','y','z'):
            if hasattr(pc,axis):setattr(pc,axis,getattr(pa,axis)+(getattr(pb,axis)-getattr(pa,axis))*t)
        # PP target position is what determines curvature. Preserve path tangent
        # for consumers that also inspect the target quaternion.
        if hasattr(result.pose.pose,'orientation'):
            yaw=math.atan2(pb.y-pa.y,pb.x-pa.x)
            q=result.pose.pose.orientation
            q.x=q.y=0.;q.z=math.sin(yaw*.5);q.w=math.cos(yaw*.5)
        return result

    def make_local_lane(self):
        """Present a forward-moving route, as Autoware's final_waypoints does.

        Pure Pursuit uses the *first* waypoint's velocity as its command. A
        permanently republished full CSV would keep commanding the initial
        zero-speed waypoint even after the recorded vehicle moves past it.
        """
        with self.progress_lock:
            lane = copy.deepcopy(self.current_route)
            if self.current_pose_xy is None:
                return lane

            waypoints = lane.waypoints
            # Never rewind to visited points when the vehicle is manually moved.
            next_idx = self.progress.next_index if self.progress is not None else 0
            first = min(max(0, next_idx - 1), len(waypoints) - 3)
            lane.waypoints = waypoints[first:]
            # Present only a bounded, forward ordered arc. Autoware's
            # Euclidean target search otherwise sees distant return branches.
            # Keep the full immutable route for progress, supervisor and RViz.
            if (self.csv_points is not None and self.progress is not None and
                    not self.progress.completed and getattr(self,'min_lookahead',None) is not None):
                lookahead=max(self.min_lookahead,self.lookahead_ratio*abs(getattr(self,'current_speed',0.)))
                end,t,arc_m=ordered_target(self.csv_points,next_idx,
                                            self.current_pose_xy[0],self.current_pose_xy[1],lookahead,yaw=getattr(self,'current_yaw',None))
                local=waypoints[first:end+1]
                if t<1.-1e-8:
                    local[-1]=self.interpolated_wp(waypoints[end-1],waypoints[end],t)
                if len(local)==2:
                    local.insert(1,self.interpolated_wp(local[0],local[-1],.5))
                lane.waypoints=local
                if hasattr(rospy,'loginfo_throttle'):
                    target=local[-1].pose.pose.position
                    rospy.loginfo_throttle(3., 'Ordered PP: next WP%d/%d, target segment WP%d, arc=%.2fm, target=(%.2f,%.2f)',
                                           next_idx+1,len(waypoints),end+1,arc_m,target.x,target.y)

            # Never select the last WPs by a global XY nearest-point search:
            # closed loops can place them beside WP01. Supervisor handles the
            # ordered final approach. Only confirmed completion forces zero here.
            if self.progress is not None and self.progress.completed:
                for wp in lane.waypoints:
                    wp.twist.twist.linear.x = 0.0
            return lane

    def on_twist(self, msg):
        self.latest_velocity = msg
        self.latest_velocity_walltime = rospy.get_time()

    def publish_ordered_twist(self,pose,yaw):
        """Publish a raw twist on a NEW valid NDT pose, never to /cmd_vel.
        Use the original source waypoint's velocity, not the interpolated
        local endpoint's velocity. GOAL completion remains supervisor-owned.
        """
        if getattr(self,'ordered_twist_pub',None) is None:return
        out=TwistStamped()
        out.header=pose.header
        with self.progress_lock:
            if self.progress is not None and not self.progress.completed:
                idx=self.progress.next_index
                first=min(max(0,idx-1),len(self.current_route.waypoints)-3)
                speed=self.current_route.waypoints[first].twist.twist.linear.x
                lookahead=max(self.min_lookahead,self.lookahead_ratio*abs(self.current_speed))
                result=ordered_pursuit(self.csv_points,idx,pose.pose.position.x,
                                       pose.pose.position.y,yaw,lookahead,speed)
                out.twist.linear.x=result['v']
                out.twist.angular.z=result['w']
                rospy.loginfo_throttle(3.,'ORDERED PP DIRECT: WP%d/%d -> WP%d, arc=%.2fm, chord=%.2fm, v=%.3fm/s, kappa=%+.3f, %s',
                    idx+1,len(self.csv_points),result['end']+1,result['arc'],result['distance'],result['v'],result['kappa'],result['reason'])
        self.ordered_twist_pub.publish(out)

    def on_pose(self, pose):
        valid_pose=False
        yaw=0.
        with self.progress_lock:
            if self.progress is None:
                self.current_pose_xy = (pose.pose.position.x, pose.pose.position.y)
            if self.progress is not None:
                import time
                active_recent = (self.route_following and self.route_active_walltime is not None and
                                 time.time()-self.route_active_walltime < 0.45)
                # Use actual orientation to disambiguate nearby opposite-direction
                # U-turn legs, while only advancing when the supervisor is ACTIVE.
                q=pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                # The ordered checkpoint is authoritative for GOAL. Do not
                # consume points from duplicate, stale or wrong-frame NDT poses.
                stamp=pose.header.stamp.to_sec()
                age=rospy.Time.now().to_sec()-stamp
                valid_pose=(pose.header.frame_id.lstrip('/')=='map' and stamp>0
                            and -0.3<=age<=1.0
                            and (self.last_progress_pose_stamp is None or
                                 stamp>self.last_progress_pose_stamp)
                            and all(not(math.isnan(v) or math.isinf(v)) for v in
                                    (pose.pose.position.x,pose.pose.position.y,yaw)))
                if valid_pose:
                    self.current_pose_xy=(pose.pose.position.x,pose.pose.position.y)
                    self.current_yaw=yaw
                    self.last_progress_pose_stamp=stamp
                changed = self.progress.update(pose.pose.position.x, pose.pose.position.y,
                                               active_recent and valid_pose, yaw=yaw)
                if changed:
                    rospy.loginfo('Waypoint progress: next WP %d/%d (local gate lateral tolerance %.2fm); active=%s',
                                  self.progress.next_index+1,len(self.csv_points),
                                  self.progress.gate_reach(self.progress.next_index),active_recent)
                    try:
                        write_progress(self.csv_path, self.csv_points, self.progress.next_index)
                    except (IOError, OSError) as exc:
                        rospy.logerr_throttle(5.0, 'Failed to save waypoint progress: %s', exc)
                    self.publish_progress_visuals()
        # Invalid CSV-mode pose cannot contaminate the next velocity estimate
        # or refresh the direct controller. Legacy LaneArray mode is unchanged.
        if self.progress is not None and not valid_pose:
            return
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
        self.current_speed=velocity.twist.linear.x if not (math.isnan(velocity.twist.linear.x) or math.isinf(velocity.twist.linear.x)) else 0.0
        self.velocity_pub.publish(velocity)
        # Do not create a fresh motor request from a stale/duplicate pose.
        # Supervisor's independent freshness watchdog handles stream loss.
        if valid_pose:
            self.publish_ordered_twist(pose,yaw)
        self.last_position = (pose.pose.position.x, pose.pose.position.y)
        self.last_stamp = pose.header.stamp


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', default=None)
    parser.add_argument('--cruise-mps',type=float,default=None)
    parser.add_argument('--lookahead-ratio',type=float,default=1.17)
    parser.add_argument('--min-lookahead',type=float,default=2.0)
    options = parser.parse_args(rospy.myargv()[1:])
    if options.cruise_mps is not None and not 0.05<=options.cruise_mps<=5.00:
        parser.error('--cruise-mps out of [0.05,5.00]')
    if not .1<=options.lookahead_ratio<=20. or not .1<=options.min_lookahead<=30.:
        parser.error('Invalid lookahead parameters')
    rospy.init_node('offline_nav_bridge', anonymous=False)
    OfflineNavBridge(csv_path=options.csv,cruise_mps=options.cruise_mps,
                     lookahead_ratio=options.lookahead_ratio,min_lookahead=options.min_lookahead)
    rospy.spin()
