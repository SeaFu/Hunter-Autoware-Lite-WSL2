#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Supervised Hunter route follower bridge (ROS Melodic/Python2).

Never launch manually without reading README. Hardware commands are disabled by
DEFAULT. --hardware enables one authorized /cmd_vel publisher, with independent
ROS freshness/route/obstacle checks and a GUI-process-coupled hold heartbeat.
VLP16 obstacle guard is supplementary ONLY; not safety-rated or a replacement
for an E-stop or an approved collision-avoidance system.
"""
from __future__ import division, print_function
import argparse
import math
import time
import threading
import rospy
import rosgraph
import tf
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String, Int32
from hunter_msgs.msg import HunterStatus
from autoware_msgs.msg import NDTStat
from route_safety import validate, nearest_route_distance, nearest_route_heading, starting_alignment, wrap
from nav_waypoint_progress import distance_to_remaining
from nav_route_corridor import route_to_base, in_corridor, corridor_edges, classify_point_region
from nav_motion_policy import PoseRateWindow, adaptive_speed_cap, ClearanceGate, required_clearance, forward_speed_cap


def yaw_of_quat(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def valid_number(x):
    return not (math.isnan(x) or math.isinf(x))


class Supervisor(object):
    RATE = 20.0
    def __init__(self, hardware=False, max_ndt_score=2.0, max_speed=0.10, pose_timeout=1.0, min_pose_hz=2.0, vehicle_width_m=0.80, max_cross_track_m=1.00, obstacle_stop_m=2.40, corridor_half_width_m=None):
        self.hardware=hardware
        self.max_ndt_score=max_ndt_score
        self.max_speed=float(max_speed)
        self.pose_timeout=float(pose_timeout)
        self.min_pose_hz=float(min_pose_hz)
        # User requested 1.00m route deviation cap; this is NOT a collision clearance.
        self.max_cross_track_m=float(max_cross_track_m)
        self.last_cross_track_full=None
        self.last_cross_track_remaining=None
        self.pose_rate=PoseRateWindow(timeout_s=pose_timeout,min_hz=min_pose_hz,recovery_s=2.0)
        self.last_ndt_header_time=None
        self.last_ndt_age=0.0
        self.lock=threading.RLock()
        self.stamps={}
        self.pose=None;self.cmd=None;self.status=None;self.route=None;self.ndt_stat=None
        self.route_info=None
        self.progress_index=0
        self.manual_since=None
        self.auto_since=None
        self.hold=False
        self.started=False; self.ever_started=False; self.complete=False; self.fault_latched=None
        self.pose_stable_since=None
        self.last_pose_jump=None
        self.stop_burst_until=0.0
        self.last_reason=None
        self.last_reason_group=None
        self.last_reason_report=0.0
        self.publishers_cache=False; self.publishers_checked_at=0.0
        self.obstacle_ok=False;self.obstacle_reason='尚未檢測到 LiDAR'
        self.clearance=ClearanceGate(clear_dwell_s=1.2,min_clear_frames=5)
        # The previous hardcoded 2.4m scan ROI cannot be used unchanged with
        # adjustable multi-m/s cruise targets. This remains a supplemental,
        # non-safety-rated pointcloud guard, not certified collision avoidance.
        # Operator chooses a STOP distance, never a raw bypass of collision
        # checking. Required model clearance sets a speed-dependent floor.
        # 1.0m model buffer is NOT a verified physical braking margin.
        self.obstacle_stop_requested_m=float(obstacle_stop_m)
        self.obstacle_stop_m=max(self.obstacle_stop_requested_m,
                                 required_clearance(self.max_speed,margin_m=1.0))
        self.immediate_front_guard_m=min(self.obstacle_stop_m,
             max(1.25,required_clearance(self.max_speed,margin_m=0.75)))
        self.obstacle_horizon_m=min(35.0,max(2.4,self.obstacle_stop_m+1.0,
                                             required_clearance(self.max_speed)+1.0))
        self.nearest_obstacle_m=None
        # User-requested extra lateral allowance is 15cm PER OUTER SIDE,
        # not a +/-15cm strip around the centerline (which would put vehicle
        # parts outside the VLP-16 obstacle ROI). Actual widest vehicle / tool
        # width must be entered by operator; 0.80m is a placeholder only.
        # This pointcloud ROI is not a certified collision/footprint checker.
        self.vehicle_width_m=float(vehicle_width_m)
        self.route_corridor_side_margin_m=0.15
        self.route_corridor_half_width=(self.vehicle_width_m / 2.0 +
                                        self.route_corridor_side_margin_m)
        # Operator-adjustable in v1.5.18; never below measured vehicle half-width
        # plus the previously agreed 0.15m allowance on either side.
        if corridor_half_width_m is not None:
            requested_half=float(corridor_half_width_m)
            if (not valid_number(requested_half) or
                    requested_half+1e-8 < self.route_corridor_half_width or
                    requested_half>1.75):
                raise ValueError('Route Corridor Half Width must be >= vehicle_width/2+0.15m and <=1.75m')
            self.route_corridor_half_width=requested_half
        self.route_corridor_side_margin_m=(self.route_corridor_half_width -
                                           self.vehicle_width_m/2.0)
        self.last_odom_speed=0.0
        self.last_pub_v=0.0; self.last_pub_time=time.time()
        self.listener=tf.TransformListener()
        self.cmd_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1) if hardware else None
        self.preview_pub=rospy.Publisher('/hunter_nav/applied_cmd',TwistStamped,queue_size=2)
        self.state_pub=rospy.Publisher('/hunter_nav/safety_state',String,queue_size=2,latch=True)
        self.route_active_pub=rospy.Publisher('/hunter_nav/route_active',Bool,queue_size=2)
        self.obstacle_marker_pub=rospy.Publisher('/hunter_nav/obstacle_markers',MarkerArray,queue_size=1)
        rospy.Subscriber('/ndt_pose',PoseStamped,self.on_pose,queue_size=2)
        rospy.Subscriber('/ndt_stat',NDTStat,self.on_ndt_stat,queue_size=2)
        rospy.Subscriber('/offline_nav/twist_cmd',TwistStamped,self.on_cmd,queue_size=2)
        rospy.Subscriber('/hunter_status',HunterStatus,self.on_status,queue_size=2)
        rospy.Subscriber('/hunter_nav/route',Path,self.on_route,queue_size=1)
        rospy.Subscriber('/hunter_nav/progress_index',Int32,self.on_progress,queue_size=1)
        rospy.Subscriber('/hunter_nav/operator_hold',Bool,self.on_hold,queue_size=2)
        rospy.Subscriber('/velodyne_points',PointCloud2,self.on_scan,queue_size=1,buff_size=2**24)
        self.pub_state('WAIT: 尚未收到 GUI 啟動心跳；本程式不具備安全認證')
        rospy.on_shutdown(self.on_shutdown)

    def arrived(self,k):
        self.stamps[k]=time.time()

    def pub_state(self,text):
        # Numeric exe_time and velocity fluctuate frame-by-frame. Do not spam
        # the GUI/CPU with a new warning every control tick. Keep the latest
        # detailed message available, and always report changes in stop reason.
        group = ('PAUSED: CLEARING' if text.startswith('PAUSED: 前方淨空確認中')
                 else 'ACTIVE' if text.startswith('ACTIVE:') else text)
        now = time.time()
        if group != self.last_reason_group or now - self.last_reason_report >= 3.0:
            rospy.logwarn('Hunter Nav: %s',text)
            self.state_pub.publish(String(text))
            self.last_reason=text
            self.last_reason_group=group
            self.last_reason_report=now

    def on_pose(self,msg):
        with self.lock:
            if msg.header.frame_id.lstrip('/')!='map':return
            # Reject repeated/out-of-order scan timestamps; they cannot count
            # as fresh localization. ROS live mode uses the same wall clock.
            sensor_stamp=msg.header.stamp.to_sec()
            if sensor_stamp>0:
                if self.last_ndt_header_time is not None and sensor_stamp<=self.last_ndt_header_time:
                    return
                header_age=rospy.Time.now().to_sec()-sensor_stamp
                if header_age>self.pose_timeout or header_age < -0.3:
                    return
                self.last_ndt_age=max(0.0,header_age)
                self.last_ndt_header_time=sensor_stamp
            xyz=(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
            if not all(valid_number(x) for x in xyz):return
            if self.pose is not None:
                p=self.pose.pose.position
                dt=time.time()-self.stamps.get('pose',0)
                displacement=math.hypot(msg.pose.position.x-p.x,msg.pose.position.y-p.y)
                # At higher legitimate speeds, one 10Hz pose update can move
                # >0.6m; use the measured vehicle / last commanded speed.
                expected_v=max(abs(self.last_odom_speed),abs(self.last_pub_v))
                allowable_jump=max(0.6,expected_v*max(0.0,dt)*1.8+0.35)
                if 0<dt<0.6 and displacement>allowable_jump:
                    if self.started:
                        # A moving vehicle must stop immediately on an XY pose jump.
                        self.fault_latched='NDT XY 位姿不合理跳點（{:.2f}m / 門檻 {:.2f}m）；停車確認定位'.format(displacement,allowable_jump)
                    else:
                        # During NDT initialization, a jump resets the continuous
                        # stability timer instead of permanently fault-latching
                        # before the vehicle has moved. Never weaken driving guard.
                        self.pose_stable_since=time.time()
                        self.last_pose_jump=self.pose_stable_since
            if self.pose_stable_since is None:
                self.pose_stable_since=time.time()
            self.pose=msg
            self.arrived('pose')
            self.pose_rate.feed(time.time())
            # After reaching the goal, the operator may take manual control,
            # return to the ORIGINAL start, and select AUTO for a new lap.
            # The completed lap can only be re-armed while in MANUAL, nearly
            # stationary, correctly oriented and with the GUI START heartbeat
            # still active. All regular checks apply again on return to AUTO.
            if (self.complete and self.route_info is not None and
                    self.status is not None and self.status.control_mode != 1 and
                    abs(self.last_odom_speed) < 0.05 and self.hold):
                aligned, _ = starting_alignment(
                    self.route_info, msg.pose.position.x, msg.pose.position.y,
                    yaw_of_quat(msg.pose.orientation))
                if aligned:
                    self.complete=False
                    self.ever_started=False
                    self.started=False
                    self.progress_index=0
                    self.pose_stable_since=time.time()
                    self.manual_since=time.time()
                    self.auto_since=None
                    self.pub_state('WAIT: 已回到起點，等待切回 AUTO 並重新確認定位／路徑')

    def on_ndt_stat(self,msg):
        with self.lock:
            if not all(valid_number(float(v)) for v in (msg.score,msg.exe_time,msg.iteration)):
                return
            self.ndt_stat=msg
            self.arrived('ndt_stat')

    def on_cmd(self,msg):
        with self.lock:
            if not all(valid_number(x) for x in (msg.twist.linear.x,msg.twist.angular.z)):return
            self.cmd=msg
            self.arrived('twist')

    def on_status(self,msg):
        with self.lock:
            self.status=msg
            self.last_odom_speed=msg.linear_velocity
            self.arrived('status')

    def on_route(self,msg):
        with self.lock:
            if msg.header.frame_id.lstrip('/')!='map':
                self.fault_latched='導航路徑不是 map 座標'
                return
            if self.started: return  # never switch paths while moving
            points=[]
            for i,p in enumerate(msg.poses):
                x=p.pose.position
                points.append({'x':x.x,'y':x.y,'z':x.z,
                    'yaw':yaw_of_quat(p.pose.orientation),
                    'velocity':0.0 if i==len(msg.poses)-1 else 1.0})
            try:
                info=validate(points)
            except ValueError as e:
                self.fault_latched='路徑無效：'+str(e);return
            self.route=points
            self.route_info=info
            self.arrived('route')

    def on_progress(self, msg):
        with self.lock:
            idx = int(msg.data)
            if self.route is not None and 0 <= idx < len(self.route):
                # Only use this progress index with the loaded route. It is a
                # monotonic next-waypoint index from the CSV bridge.
                self.progress_index = idx

    def on_hold(self,msg):
        with self.lock:
            self.hold=bool(msg.data)
            self.arrived('hold')
            if not self.hold and self.started:
                self.started=False
                self.stop_burst_until=time.time()+0.60
                self.pub_state('STOP: GUI 啟動心跳已停止；本輪導航結束')
                self.fault_latched='GUI 控制心跳中斷；本輪需重新啟動測試'

    def on_scan(self,msg):
        # Avoid blocking the control loop: PointCloud2 parsing is on callback thread.
        now=time.time()
        with self.lock:
            self.arrived('scan')
            if now-self.stamps.get('scan_checked',0)<0.24:
                return
            self.arrived('scan_checked')
        frame=msg.header.frame_id.strip('/')
        if not frame:
            with self.lock:
                self.obstacle_ok=self.clearance.update(False,time.time())
                self.obstacle_reason='LiDAR frame_id 空白'
            return
        try:
            (trans,quat)=self.listener.lookupTransform('base_link',frame,rospy.Time(0))
            # Rotate each sampled xyz into base_link, then apply translation.
            qx,qy,qz,qw=quat
            r11=1-2*(qy*qy+qz*qz); r12=2*(qx*qy-qz*qw); r13=2*(qx*qz+qy*qw)
            r21=2*(qx*qy+qz*qw); r22=1-2*(qx*qx+qz*qz); r23=2*(qy*qz-qx*qw)
            r31=2*(qx*qz-qy*qw); r32=2*(qy*qz+qx*qw); r33=1-2*(qx*qx+qy*qy)
            checked=0
            distance_bins={}
            side_bins={}
            bin_width=0.75
            with self.lock:
                route_snapshot=self.route
                progress_snapshot=self.progress_index
                pose_snapshot=self.pose
            corridor_path=None
            if route_snapshot is not None and pose_snapshot is not None:
                p=pose_snapshot.pose.position
                corridor_path=route_to_base(route_snapshot, progress_snapshot,
                                            p.x, p.y, yaw_of_quat(pose_snapshot.pose.orientation),
                                            self.obstacle_horizon_m)
            # Safety fallback: if route geometry is unavailable, use the old
            # conservative rectangular region. Never silently declare CLEAR.
            corridor_ready=corridor_path is not None
            for i,point in enumerate(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)):
                if i % 6:
                    continue
                x,y,z=point
                bx=r11*x+r12*y+r13*z+trans[0]
                by=r21*x+r22*y+r23*z+trans[1]
                bz=r31*x+r32*y+r33*z+trans[2]
                checked+=1
                if not (0.50<bx<self.obstacle_horizon_m and 0.10<bz<1.65):
                    continue
                if corridor_ready:
                    # First use the NEXT portion of the planned route. A wall
                    # far to the side of that route is diagnostic, not an
                    # automatic obstacle solely because x<2.4m. Still retain
                    # a short, body-aligned imminent-collision region, since
                    # route tracking error may place the chassis off-path.
                    # WALLS WITHIN body/route are never exempted.
                    region,projection=classify_point_region(
                        bx,by,corridor_path,self.route_corridor_half_width,
                        self.immediate_front_guard_m,self.obstacle_horizon_m)
                    if region=='invalid':
                        continue
                    if region=='body':
                        cell=('body',int(bx / bin_width),
                              int((by+self.route_corridor_half_width) / 0.30))
                        bins=distance_bins
                    else:
                        if projection is None:
                            continue
                        path_s=projection[1]
                        lateral=projection[2]
                        # 2D bins prevent separate surfaces in a range strip
                        # from being conflated into one obstacle cluster.
                        cell=(int(path_s / bin_width),int(lateral / 0.30))
                        bins=distance_bins if region=='route' else side_bins
                else:
                    if abs(by)>=1.20:
                        continue
                    cell=(int(bx / bin_width),int((by+1.2)/0.30))
                    bins=distance_bins
                if cell not in bins:
                    # Region is diagnostic metadata only; it must never change
                    # which returns count as hazards or the stop/clear decision.
                    bins[cell]=[0,bx,by,bz,region if corridor_ready else 'fallback',
                                projection[0] if corridor_ready and projection is not None else float('nan')]
                current=bins[cell]
                current[0]+=1
                if bx<current[1]:
                    current[1],current[2],current[3]=bx,by,bz
                    current[4]=region if corridor_ready else 'fallback'
                    current[5]=projection[0] if corridor_ready and projection is not None else float('nan')
            candidates=[item for item in distance_bins.values() if item[0]>=3]
            nearest=min(candidates,key=lambda t:t[1]) if candidates else None
            side_candidates=[item for item in side_bins.values() if item[0]>=3]
            nearest_side=min(side_candidates,key=lambda t:t[1]) if side_candidates else None
            closest=nearest[1] if nearest is not None else None
            marker_array=MarkerArray()
            clear_marker=Marker()
            clear_marker.header.frame_id='base_link'
            clear_marker.header.stamp=rospy.Time.now()
            clear_marker.action=Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            if corridor_ready:
                def add_path_marker(identifier, points, r,g,b, label):
                    if len(points)<2:
                        return
                    line=Marker()
                    line.header=clear_marker.header
                    line.ns=label
                    line.id=identifier
                    line.type=Marker.LINE_STRIP
                    line.action=Marker.ADD
                    line.pose.orientation.w=1.0
                    line.scale.x=0.055
                    line.color.r=r
                    line.color.g=g
                    line.color.b=b
                    line.color.a=0.92
                    for x,y in points:
                        pt=Point()
                        pt.x=x;pt.y=y;pt.z=0.10
                        line.points.append(pt)
                    marker_array.markers.append(line)
                left,right=corridor_edges(corridor_path,self.route_corridor_half_width)
                add_path_marker(10,corridor_path,0.10,0.80,0.98,'path_corridor_center')
                add_path_marker(11,left,0.30,0.70,1.00,'path_corridor_left')
                add_path_marker(12,right,0.30,0.70,1.00,'path_corridor_right')
                # The independent body-aligned rectangle is often responsible
                # for stopping at a turn even if the candidate is outside
                # the curved route strip.  This extra outline is for RViz
                # diagnosis ONLY; all existing stop conditions remain active.
                near_front_rect=[(0.50,-self.route_corridor_half_width),
                      (self.immediate_front_guard_m,-self.route_corridor_half_width),
                      (self.immediate_front_guard_m,+self.route_corridor_half_width),
                      (0.50,+self.route_corridor_half_width),
                      (0.50,-self.route_corridor_half_width)]
                add_path_marker(13,near_front_rect,1.00,0.35,0.14,'independent_body_rectangle')
                body_hint=Marker()
                body_hint.header=clear_marker.header
                body_hint.ns='body_rectangle_explanation'
                body_hint.id=14
                body_hint.type=Marker.TEXT_VIEW_FACING
                body_hint.action=Marker.ADD
                body_hint.pose.orientation.w=1.0
                body_hint.pose.position.x=self.immediate_front_guard_m
                body_hint.pose.position.y=-self.route_corridor_half_width-0.15
                body_hint.pose.position.z=0.18
                body_hint.scale.z=0.20
                body_hint.color.r=1.0;body_hint.color.g=0.55;body_hint.color.b=0.25;body_hint.color.a=0.90
                body_hint.text='INDEPENDENT BODY GUARD: x<=%.2fm, half-width=%.2fm' % (
                    self.immediate_front_guard_m,self.route_corridor_half_width)
                marker_array.markers.append(body_hint)
            if nearest is not None:
                marker=Marker()
                marker.header=clear_marker.header
                marker.ns='nearest_obstacle_candidate'
                marker.id=1
                marker.type=Marker.SPHERE
                marker.action=Marker.ADD
                marker.pose.position.x=nearest[1]
                marker.pose.position.y=nearest[2]
                marker.pose.position.z=nearest[3]
                marker.pose.orientation.w=1.0
                marker.scale.x=marker.scale.y=marker.scale.z=0.30
                marker.color.r=1.0
                marker.color.g=0.15
                marker.color.b=0.05
                marker.color.a=1.0
                marker_array.markers.append(marker)
                label=Marker()
                label.header=clear_marker.header
                label.ns='obstacle_distance_label'
                label.id=2
                label.type=Marker.TEXT_VIEW_FACING
                label.action=Marker.ADD
                label.pose.position.x=nearest[1]
                label.pose.position.y=nearest[2]
                label.pose.position.z=nearest[3]+0.5
                label.pose.orientation.w=1.0
                label.scale.z=0.32
                label.color.r=1.0
                label.color.g=0.45
                label.color.b=0.2
                label.color.a=1.0
                label.text='{} x={:.2f} y={:+.2f}m route_dist={:.2f}m'.format(
                    'BODY_RECT' if nearest[4]=='body' else
                    'ROUTE_STRIP' if nearest[4]=='route' else 'FALLBACK_RECT',
                    nearest[1],nearest[2],nearest[5])
                marker_array.markers.append(label)
            if nearest_side is not None:
                # Amber is outside the provisional route corridor (diagnostic
                # only). It is NOT a guarantee the side is safe to pass.
                side=Marker()
                side.header=clear_marker.header
                side.ns='out_of_corridor_candidate'
                side.id=20
                side.type=Marker.SPHERE
                side.action=Marker.ADD
                side.pose.position.x=nearest_side[1]
                side.pose.position.y=nearest_side[2]
                side.pose.position.z=nearest_side[3]
                side.pose.orientation.w=1.0
                side.scale.x=side.scale.y=side.scale.z=0.20
                side.color.r=1.0;side.color.g=0.77;side.color.b=0.08;side.color.a=0.90
                marker_array.markers.append(side)
                side_label=Marker()
                side_label.header=clear_marker.header
                side_label.ns='side_wall_diagnostic_label'
                side_label.id=21
                side_label.type=Marker.TEXT_VIEW_FACING
                side_label.action=Marker.ADD
                side_label.pose.position.x=nearest_side[1]
                side_label.pose.position.y=nearest_side[2]
                side_label.pose.position.z=nearest_side[3]+0.4
                side_label.pose.orientation.w=1.0
                side_label.scale.z=0.26
                side_label.color.r=1.0;side_label.color.g=0.75;side_label.color.b=0.1;side_label.color.a=1.0
                side_label.text='SIDE / likely wall (outside planned ROI)'
                marker_array.markers.append(side_label)
            self.obstacle_marker_pub.publish(marker_array)
            with self.lock:
                self.nearest_obstacle_m=closest
                clear=(checked>=100 and corridor_ready and
                       (closest is None or closest>self.obstacle_stop_m))
                self.obstacle_ok=self.clearance.update(clear,time.time())
                self.obstacle_reason=(
                    '行駛範圍候選 [{}] x={:.2f}m y={:+.2f}m 距剩餘路線={:.2f}m（停車門檻 {:.2f}m；車頭矩形長 {:.2f}m）'.format(
                        'BODY_RECT' if nearest[4]=='body' else
                        'ROUTE_STRIP' if nearest[4]=='route' else 'FALLBACK_RECT',
                        nearest[1],nearest[2],nearest[5],self.obstacle_stop_m,self.immediate_front_guard_m)
                    if nearest is not None and closest<=self.obstacle_stop_m else
                    '點雲有效點不足' if checked<100 else
                    '路徑走廊尚未建立，暫停導航' if not corridor_ready else
                    '路徑走廊淨空確認中 {:.1f}/1.2s'.format(self.clearance.elapsed(time.time()))
                    if not self.obstacle_ok else 'CLEAR')
        except Exception as e:
            with self.lock:
                self.obstacle_ok=self.clearance.update(False,time.time())
                self.obstacle_reason='無法檢查 LiDAR/TF：'+str(e)[:120]

    def publishers_ok(self):
        if not self.hardware:return True
        now=time.time()
        if now-self.publishers_checked_at<0.30:
            return self.publishers_cache
        self.publishers_checked_at=now
        self.publishers_cache=False
        try:
            master=rosgraph.Master('/hunter_nav_supervisor')
            pubs,_,_=master.getSystemState()
            ours='/hunter_nav_supervisor'
            for topic,nodes in pubs:
                if topic=='/cmd_vel':
                    others=[x for x in nodes if x!=ours]
                    self.publishers_cache=(not others)
                    return self.publishers_cache
        except Exception:return False
        return False

    def healthy(self,now):
        if self.fault_latched:return 'STOP: '+self.fault_latched
        if self.complete:return 'GOAL: 路徑終點，已停止'
        if self.pose_stable_since is None or now-self.pose_stable_since<2.5:
            return 'WAIT: NDT XY 位姿需連續穩定 2.5 秒後才可啟動'
        if self.pose_rate.age(now)>=self.pose_timeout:
            self.pose_rate.recovery_required=True
            return 'PAUSED: 定位已超過 {:.1f}s 沒有新位姿'.format(self.pose_timeout)
        if not self.pose_rate.ready(now):
            return 'PAUSED: 等待連續 2 秒、至少 {:.1f} Hz 的有效定位'.format(self.min_pose_hz)
        scan_timeout=(0.45 if self.max_speed>1.0 else 1.0)
        for name,threshold in (('ndt_stat',1.0),('twist',1.0),('status',0.50),('scan',scan_timeout),('scan_checked',scan_timeout),('hold',0.30),('route',1800)):
            if now-self.stamps.get(name,0)>threshold:return 'PAUSED: {} 資料逾時'.format(name)
        if self.status.fault_code!=0 or self.status.base_state!=0:
            return 'STOP: Hunter fault/base_state = {}/{}'.format(self.status.fault_code,self.status.base_state)
        # RC mode must not be bypassed to command a vehicle. Enable AUTO via the
        # approved remote procedure, then confirm the actual control_mode value.
        if self.hardware and self.status.control_mode!=1:
            self.auto_since=None
            self.manual_since=now if self.manual_since is None else self.manual_since
            return 'PAUSED: Hunter 已切換手動控制（control_mode={}）；恢復 AUTO 後需再次確認路徑'.format(self.status.control_mode)
        if self.hardware and self.manual_since is not None:
            if self.auto_since is None:
                self.auto_since=now
            if now-self.auto_since < 2.0:
                return 'PAUSED: AUTO 已恢復，等待 2 秒定位／路徑確認'
            if self.progress_index > 0 and self.route is not None and self.pose is not None:
                q=self.pose.pose.orientation
                dist,head=distance_to_remaining(self.route,self.progress_index,
                                                 self.pose.pose.position.x,self.pose.pose.position.y,
                                                 max_forward_segments=3)
                if dist>0.50 or abs(wrap(yaw_of_quat(q)-head))>0.40:
                    return 'PAUSED: 恢復 AUTO 時未對齊剩餘路徑（距離 {:.2f}m）'.format(dist)
            self.manual_since=None
            self.auto_since=None
        if self.ndt_stat.score<0 or self.ndt_stat.score>self.max_ndt_score:
            return 'STOP: NDT fitness score {:.3f} 超出上限 {:.2f}'.format(self.ndt_stat.score,self.max_ndt_score)
        if self.ndt_stat.iteration>=29 or self.ndt_stat.exe_time<=0:
            return 'PAUSED: NDT 未收斂或運算時間無效'
        # exe_time is now used to scale speed; it alone does not stop motion.
        if not self.obstacle_ok:return 'PAUSED: '+self.obstacle_reason
        if self.route is None:return 'STOP: 尚無有效 Waypoint 路徑'
        if not self.publishers_ok():return 'STOP: /cmd_vel 另有 Publisher 或無法驗證'
        if not self.hold:return 'WAIT: GUI 控制心跳尚未就緒'
        pose=self.pose.pose
        yaw=yaw_of_quat(pose.orientation)
        if not self.ever_started:
            if self.progress_index > 0:
                cross_rem, heading_rem = distance_to_remaining(
                    self.route, self.progress_index, pose.position.x, pose.position.y, max_forward_segments=3)
                if cross_rem > 0.50:
                    return 'STOP: 距離尚未完成的 Waypoint 路徑 {:.2f}m；需移到剩餘路徑附近或重設進度'.format(cross_rem)
                if abs(wrap(yaw-heading_rem)) > 0.40:
                    return 'STOP: 恢復導航時車頭與剩餘路徑方向不符'
            else:
                ok,explain=starting_alignment(self.route_info,pose.position.x,pose.position.y,yaw)
                if not ok:return 'STOP: '+explain
        remaining_route=self.route[max(0,self.progress_index-1):] if self.progress_index>0 else self.route
        cross_remaining=nearest_route_distance(remaining_route,pose.position.x,pose.position.y)
        cross_full=nearest_route_distance(self.route,pose.position.x,pose.position.y)
        self.last_cross_track_full=cross_full
        self.last_cross_track_remaining=cross_remaining
        if cross_remaining>self.max_cross_track_m:
            # Both distances are logged, so a prematurely advanced waypoint
            # index cannot masquerade as a genuine full-route tracking error.
            if cross_full + 0.20 < cross_remaining:
                return ('STOP: Waypoint 進度與實際位置不一致：'
                        'WP{}/{}，剩餘路徑偏差 {:.2f}m，完整路徑偏差 {:.2f}m；'
                        '請停車檢查進度，不自動跳回舊路線').format(
                            self.progress_index+1,len(self.route),cross_remaining,cross_full)
            return ('STOP: 偏離路徑 {:.2f}m > {:.2f}m；'
                    '完整路徑 {:.2f}m，WP{}/{}').format(
                        cross_remaining,self.max_cross_track_m,cross_full,
                        self.progress_index+1,len(self.route))
        # Fail closed if the bridge never consumes WP01 after we have travelled
        # well past the first segment.  Otherwise Pure Pursuit may keep using
        # the entire (possibly folded) U-turn route with stale progress while
        # /cmd_vel continues to command forward motion.
        if self.ever_started and self.progress_index==0 and len(self.route)>=2:
            first,second=self.route[0],self.route[1]
            sx=second['x']-first['x']; sy=second['y']-first['y']
            seg_length=math.hypot(sx,sy)
            if seg_length>1e-6:
                along=((pose.position.x-first['x'])*sx+
                       (pose.position.y-first['y'])*sy)/seg_length
                if along>seg_length+0.20:
                    return ('STOP: Waypoint 進度卡在 WP01：沿起始路段已行進 {:.2f}m，'
                            '但尚未收到 WP02 進度；停止輸出，請檢查導航進度回報').format(along)
        heading=nearest_route_heading(remaining_route,pose.position.x,pose.position.y)
        if abs(wrap(yaw-heading))>0.70:
            return 'STOP: 車頭方向與最近路徑段差 {:.2f}rad'.format(abs(wrap(yaw-heading)))
        dx=pose.position.x-self.route[-1]['x'];dy=pose.position.y-self.route[-1]['y']
        if math.hypot(dx,dy)<0.55:
            self.complete=True
            return 'GOAL: 距離終點小於 0.55m，已停止'
        if self.cmd.twist.linear.x<=0:
            return 'STOP: Pure Pursuit 速度非正（可能正在等待路徑）'
        return None

    def publish_zero(self):
        if self.cmd_pub:self.cmd_pub.publish(Twist())
        t=TwistStamped();t.header.stamp=rospy.Time.now()
        self.preview_pub.publish(t)
        self.last_pub_v=0.0
        self.last_pub_time=time.time()

    def tick(self,_evt):
        now=time.time()
        with self.lock:
            err=self.healthy(now)
            if err:
                if self.started:
                    self.started=False
                # Only structural/control integrity faults are latched. A clear
                # obstacle or a short pose gap is a PAUSE, not a new run.
                fatal=(err.startswith('STOP: Hunter fault/') or
                       err.startswith('STOP: /cmd_vel 另有') or
                       err.startswith('STOP: 偏離路徑') or
                       err.startswith('STOP: Waypoint 進度與實際位置不一致') or
                       err.startswith('STOP: Waypoint 進度卡在 WP01') or
                       err.startswith('STOP: 車頭方向'))
                if self.ever_started and fatal:
                    self.fault_latched=err
                # Keep sending zero while stopped: never rely on a single pulse.
                self.publish_zero()
                self.route_active_pub.publish(Bool(False))
                self.pub_state(err)
                return
            msg=self.cmd.twist
            hz=self.pose_rate.hz(now)
            limit=adaptive_speed_cap(self.max_speed,self.ndt_stat.exe_time,hz)
            # A pose that is approaching the 1s timeout must slow down already;
            # do not run at full target speed until the timeout fires.
            pose_age=self.pose_rate.age(now)
            if pose_age>0.20:
                limit=min(limit,self.max_speed*max(0.0,(self.pose_timeout-pose_age)/
                                                     max(0.05,self.pose_timeout-0.20)))
            # Proportional slowdown for an obstacle farther than the immediate
            # stop zone. Actual vehicle braking performance is NOT known here.
            if self.nearest_obstacle_m is not None:
                limit=min(limit,forward_speed_cap(self.nearest_obstacle_m,self.max_speed))
            requested_v=max(0.0,msg.linear.x)
            # Preserve the curvature of Pure Pursuit when limiting speed.
            curvature=(msg.angular.z/max(requested_v,0.05))
            if abs(curvature)>1e-6:
                limit=min(limit,math.sqrt(0.80/abs(curvature)),0.50/abs(curvature))
            # Approach the final waypoint with a modeled deceleration envelope.
            goal_dx=self.pose.pose.position.x-self.route[-1]['x']
            goal_dy=self.pose.pose.position.y-self.route[-1]['y']
            remaining=max(0.0,math.hypot(goal_dx,goal_dy)-0.55)
            limit=min(limit,math.sqrt(2.0*0.65*remaining))
            v=min(limit,requested_v)
            if v<=0.005:
                self.publish_zero()
                self.route_active_pub.publish(Bool(False))
                self.pub_state('PAUSED: 目前允許速度過低（定位／前方距離／彎道／終點），保持停車')
                return
            dt=min(0.15,max(0.0,now-self.last_pub_time))
            # Ramp up after initial start and after obstacle clearance.
            # Immediate stop requests are deliberately not delayed by ramping.
            v=min(v,self.last_pub_v+0.50*dt)
            w=curvature*v
            w=max(-0.50,min(0.50,w))
            self.last_pub_v=v
            self.last_pub_time=now
            out=Twist();out.linear.x=v;out.angular.z=w
            if self.cmd_pub:self.cmd_pub.publish(out)
            self.started=True
            self.ever_started=True
            self.route_active_pub.publish(Bool(True))
            report=TwistStamped();report.header.stamp=rospy.Time.now();report.twist=out
            self.preview_pub.publish(report)
            obstacle_text=('無近距離候選' if self.nearest_obstacle_m is None else
                           '{:.1f}m'.format(self.nearest_obstacle_m))
            self.pub_state(('ACTIVE: WP{}/{}; cmd={:.2f}/{:.2f}m/s; forward={}; '
                            'dev_full={:.2f}m; dev_rem={:.2f}m (limit={:.2f}m); '
                            'kappa_req={:+.3f} 1/m; kappa_out={:+.3f} 1/m; '
                            'yaw_req={:+.3f}; yaw_out={:+.3f}rad/s; pose={:.1f}Hz; NDT={:.0f}ms').format(
                self.progress_index+1,len(self.route),v,self.max_speed,obstacle_text,
                self.last_cross_track_full if self.last_cross_track_full is not None else float('nan'),
                self.last_cross_track_remaining if self.last_cross_track_remaining is not None else float('nan'),
                self.max_cross_track_m,curvature,w/max(v,0.005),msg.angular.z,w,hz,self.ndt_stat.exe_time))

    def on_shutdown(self):
        for _ in range(8):
            try:
                self.route_active_pub.publish(Bool(False))
                if self.cmd_pub:self.cmd_pub.publish(Twist())
                time.sleep(0.04)
            except Exception:
                break


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--hardware',action='store_true',help='Allow actual /cmd_vel publishing')
    ap.add_argument('--max-ndt-score',type=float,default=2.0,help='NDT fitness score guard')
    ap.add_argument('--max-speed',type=float,default=0.10,help='max route speed (m/s)')
    ap.add_argument('--vehicle-width-m',type=float,default=0.80,help='Measured widest vehicle+attachment external width in meters; +0.15m per side')
    ap.add_argument('--corridor-half-width-m',type=float,default=None,help='Operator route ROI half-width, meters; min=measured vehicle width / 2 + 0.15m')
    ap.add_argument('--max-cross-track-m',type=float,default=1.00,help='Stop threshold for remaining-route cross-track error; default 1.00m')
    ap.add_argument('--obstacle-stop-m',type=float,default=2.40,help='Operator forward stopping distance, meters; may be increased by modeled speed clearance')
    ap.add_argument('--pose-timeout',type=float,default=1.0,help='pause after no new pose for this long (s)')
    ap.add_argument('--min-pose-hz',type=float,default=2.0,help='minimum validated recovery pose rate')
    opts=ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('hunter_nav_supervisor',anonymous=False)
    if opts.max_ndt_score<=0 or opts.max_ndt_score>10:ap.error("--max-ndt-score 需在 (0,10] 範圍")
    if not 0.05<=opts.max_speed<=5.00:ap.error('--max-speed 必須在 [0.05,5.00] m/s')
    if not 0.60<=opts.vehicle_width_m<=2.50:ap.error('--vehicle-width-m 必須在 [0.60,2.50] m；請量測最外側寬度')
    if opts.corridor_half_width_m is not None and not (opts.vehicle_width_m / 2.0 + 0.15 - 1e-8 <= opts.corridor_half_width_m <= 1.75):
        ap.error('--corridor-half-width-m 必須 >= 車體外寬/2 + 0.15m，且 <= 1.75m')
    if not 0.30<=opts.max_cross_track_m<=1.00:ap.error('--max-cross-track-m 必須在 [0.30,1.00] m')
    if not 1.50<=opts.obstacle_stop_m<=10.0:ap.error('--obstacle-stop-m 必須在 [1.50,10.00] m')
    if not 0.8<=opts.pose_timeout<=1.5:ap.error('--pose-timeout 必須在 [0.8,1.5] s')
    if not 2.0<=opts.min_pose_hz<=10.0:ap.error('--min-pose-hz 必須在 [2,10] Hz')
    x=Supervisor(hardware=opts.hardware,max_ndt_score=opts.max_ndt_score,
                 max_speed=opts.max_speed,pose_timeout=opts.pose_timeout,min_pose_hz=opts.min_pose_hz,
                 vehicle_width_m=opts.vehicle_width_m,max_cross_track_m=opts.max_cross_track_m,
                 obstacle_stop_m=opts.obstacle_stop_m,
                 corridor_half_width_m=opts.corridor_half_width_m)
    rospy.Timer(rospy.Duration(1.0/x.RATE),x.tick)
    rospy.logwarn('SUPERVISOR OUTPUT MODE: %s', 'REAL HARDWARE' if opts.hardware else 'DRY RUN, /cmd_vel DISABLED')
    rospy.logwarn('HUNTER CORRIDOR: half=%.2fm total=%.2fm; measured vehicle=%.2fm; side margin=%.2fm each (not collision certified)',x.route_corridor_half_width,x.route_corridor_half_width*2,x.vehicle_width_m,x.route_corridor_side_margin_m)
    rospy.logwarn('ROUTE DEVIATION: remaining-route stop limit=%.2fm; full-route and remaining-route errors are logged separately',x.max_cross_track_m)
    rospy.logwarn('OBSTACLE STOP: GUI requested=%.2fm effective=%.2fm (speed/braking model floor); near-body guard=%.2fm; side ROI is diagnostic, walls in swept route still STOP',x.obstacle_stop_requested_m,x.obstacle_stop_m,x.immediate_front_guard_m)
    rospy.spin()

if __name__=='__main__':main()
