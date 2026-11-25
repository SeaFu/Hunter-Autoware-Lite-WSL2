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
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, String
from hunter_msgs.msg import HunterStatus
from autoware_msgs.msg import NDTStat
from route_safety import validate, nearest_route_distance, nearest_route_heading, starting_alignment, wrap
from nav_motion_policy import PoseRateWindow, adaptive_speed_cap, ClearanceGate, required_clearance, forward_speed_cap


def yaw_of_quat(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def valid_number(x):
    return not (math.isnan(x) or math.isinf(x))


class Supervisor(object):
    RATE = 20.0
    def __init__(self, hardware=False, max_ndt_score=2.0, max_speed=0.10, pose_timeout=1.0, min_pose_hz=2.0):
        self.hardware=hardware
        self.max_ndt_score=max_ndt_score
        self.max_speed=float(max_speed)
        self.pose_timeout=float(pose_timeout)
        self.min_pose_hz=float(min_pose_hz)
        self.pose_rate=PoseRateWindow(timeout_s=pose_timeout,min_hz=min_pose_hz,recovery_s=2.0)
        self.last_ndt_header_time=None
        self.last_ndt_age=0.0
        self.lock=threading.RLock()
        self.stamps={}
        self.pose=None;self.cmd=None;self.status=None;self.route=None;self.ndt_stat=None
        self.route_info=None
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
        self.obstacle_horizon_m=min(35.0,max(2.4,required_clearance(self.max_speed)+1.0))
        self.nearest_obstacle_m=None
        self.last_odom_speed=0.0
        self.last_pub_v=0.0; self.last_pub_time=time.time()
        self.listener=tf.TransformListener()
        self.cmd_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1) if hardware else None
        self.preview_pub=rospy.Publisher('/hunter_nav/applied_cmd',TwistStamped,queue_size=2)
        self.state_pub=rospy.Publisher('/hunter_nav/safety_state',String,queue_size=2,latch=True)
        rospy.Subscriber('/ndt_pose',PoseStamped,self.on_pose,queue_size=2)
        rospy.Subscriber('/ndt_stat',NDTStat,self.on_ndt_stat,queue_size=2)
        rospy.Subscriber('/offline_nav/twist_cmd',TwistStamped,self.on_cmd,queue_size=2)
        rospy.Subscriber('/hunter_status',HunterStatus,self.on_status,queue_size=2)
        rospy.Subscriber('/hunter_nav/route',Path,self.on_route,queue_size=1)
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
            bin_width=0.75
            # Search farther ahead when the user requests a higher speed.
            # Each distance bin requires >=3 sampled hits, suppressing single
            # noisy returns from causing a sudden high-speed stop.
            for i,p in enumerate(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)):
                if i % 6:continue
                x,y,z=p
                bx=r11*x+r12*y+r13*z+trans[0]
                by=r21*x+r22*y+r23*z+trans[1]
                bz=r31*x+r32*y+r33*z+trans[2]
                checked+=1
                if 0.50<bx<self.obstacle_horizon_m and abs(by)<1.20 and 0.10<bz<1.65:
                    cell=int(bx/bin_width)
                    distance_bins[cell]=distance_bins.get(cell,0)+1
            candidates=[key*bin_width for key,n in distance_bins.items() if n>=3]
            closest=min(candidates) if candidates else None
            with self.lock:
                self.nearest_obstacle_m=closest
                clear=(checked>=100 and (closest is None or closest>2.40))
                self.obstacle_ok=self.clearance.update(clear,time.time())
                self.obstacle_reason=(
                    '前方近距離障礙候選 {:.1f}m'.format(closest) if closest is not None and closest<=2.4 else
                    '點雲有效點不足' if checked<100 else
                    '前方淨空確認中 {:.1f}/1.2s'.format(self.clearance.elapsed(time.time()))
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
            return 'STOP: Hunter 尚非 AUTONOMOUS 控制模式（control_mode={}）'.format(self.status.control_mode)
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
            ok,explain=starting_alignment(self.route_info,pose.position.x,pose.position.y,yaw)
            if not ok:return 'STOP: '+explain
        cross=nearest_route_distance(self.route,pose.position.x,pose.position.y)
        if cross>0.45:return 'STOP: 偏離路徑 {:.2f}m > 0.45m'.format(cross)
        heading=nearest_route_heading(self.route,pose.position.x,pose.position.y)
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
                       err.startswith('STOP: Hunter 尚非') or
                       err.startswith('STOP: /cmd_vel 另有') or
                       err.startswith('STOP: 偏離路徑') or
                       err.startswith('STOP: 車頭方向'))
                if self.ever_started and fatal:
                    self.fault_latched=err
                # Keep sending zero while stopped: never rely on a single pulse.
                self.publish_zero()
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
            report=TwistStamped();report.header.stamp=rospy.Time.now();report.twist=out
            self.preview_pub.publish(report)
            obstacle_text=('無近距離候選' if self.nearest_obstacle_m is None else
                           '{:.1f}m'.format(self.nearest_obstacle_m))
            self.pub_state('ACTIVE: cmd={:.2f} / target={:.2f}m/s; forward={}; yaw={:.3f}; pose={:.1f}Hz; NDT={:.0f}ms'.format(
                v,self.max_speed,obstacle_text,w,hz,self.ndt_stat.exe_time))

    def on_shutdown(self):
        if self.cmd_pub:
            for _ in range(8):
                try:self.cmd_pub.publish(Twist());time.sleep(0.04)
                except Exception:break


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--hardware',action='store_true',help='Allow actual /cmd_vel publishing')
    ap.add_argument('--max-ndt-score',type=float,default=2.0,help='NDT fitness score guard')
    ap.add_argument('--max-speed',type=float,default=0.10,help='max route speed (m/s)')
    ap.add_argument('--pose-timeout',type=float,default=1.0,help='pause after no new pose for this long (s)')
    ap.add_argument('--min-pose-hz',type=float,default=2.0,help='minimum validated recovery pose rate')
    opts=ap.parse_args(rospy.myargv()[1:])
    rospy.init_node('hunter_nav_supervisor',anonymous=False)
    if opts.max_ndt_score<=0 or opts.max_ndt_score>10:ap.error("--max-ndt-score 需在 (0,10] 範圍")
    if not 0.05<=opts.max_speed<=5.00:ap.error('--max-speed 必須在 [0.05,5.00] m/s')
    if not 0.8<=opts.pose_timeout<=1.5:ap.error('--pose-timeout 必須在 [0.8,1.5] s')
    if not 2.0<=opts.min_pose_hz<=10.0:ap.error('--min-pose-hz 必須在 [2,10] Hz')
    x=Supervisor(hardware=opts.hardware,max_ndt_score=opts.max_ndt_score,
                 max_speed=opts.max_speed,pose_timeout=opts.pose_timeout,min_pose_hz=opts.min_pose_hz)
    rospy.Timer(rospy.Duration(1.0/x.RATE),x.tick)
    rospy.logwarn('SUPERVISOR OUTPUT MODE: %s', 'REAL HARDWARE' if opts.hardware else 'DRY RUN, /cmd_vel DISABLED')
    rospy.spin()

if __name__=='__main__':main()
