#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read one fresh NDT pose; report starting XY/yaw alignment, no motor commands."""
from __future__ import print_function
import argparse
import json
import math
import rospy
from geometry_msgs.msg import PoseStamped
from route_safety import load_route,validate,wrap
from nav_waypoint_progress import read_progress, distance_to_remaining


def report(payload):
    print('NAV_STARTPOINT_RESULT='+json.dumps(payload,ensure_ascii=True,sort_keys=True))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--csv',required=True)
    args=p.parse_args(rospy.myargv()[1:])
    try:
        points=load_route(args.csv)
        info=validate(points)
        next_idx=read_progress(args.csv,points)
    except (OSError,ValueError) as e:
        report({'ok':False,'type':'error','message':'Waypoint CSV 不能使用：'+str(e)})
        return
    rospy.init_node('hunter_startpoint_precheck',anonymous=True,disable_signals=True)
    try:
        pose=rospy.wait_for_message('/ndt_pose',PoseStamped,timeout=4.0)
    except rospy.ROSException:
        report({'ok':False,'type':'error','message':'4 秒內未收到 /ndt_pose；請先恢復 NDT Matching'})
        return
    if pose.header.frame_id.lstrip('/')!='map':
        report({'ok':False,'type':'error','message':'/ndt_pose 不在 map frame'})
        return
    age=(rospy.Time.now()-pose.header.stamp).to_sec()
    if not (0<=age<=0.6):
        report({'ok':False,'type':'error','message':'/ndt_pose 時戳太舊或時間不同步：age={:.2f}s'.format(age)})
        return
    p=pose.pose.position
    q=pose.pose.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    sx,sy,syaw=info['start']
    d=math.hypot(p.x-sx,p.y-sy)
    dyaw=abs(wrap(yaw-syaw))
    if next_idx > 0:
        d, path_heading=distance_to_remaining(points,next_idx,p.x,p.y,max_forward_segments=3)
        dyaw=abs(wrap(yaw-path_heading))
        sx,sy=points[next_idx]['x'],points[next_idx]['y']
    base={'distance_m':round(d,4),'limit_m':0.50,'current_xy':[round(p.x,4),round(p.y,4)],
          'start_xy':[round(sx,4),round(sy,4)],'yaw_difference_rad':round(dyaw,4),
          'resume':bool(next_idx),'next_index':next_idx,'waypoint_count':len(points)}
    if d>0.50:
        if next_idx > 0:
            message=('距離剩餘路徑 {:.2f}m；請移到 WP{} 附近，或停止導航後按「重設進度／從起點重跑」'
                     .format(d,next_idx+1))
        else:
            message='距離 Waypoint 起點 {:.2f}m，請先遙控移到起點附近'.format(d)
        base.update({'ok':False,'type':'too_far','message':message})
    elif dyaw>0.40:
        base.update({'ok':False,'type':'yaw_mismatch','message':'車頭與起始方向相差 {:.2f}rad（門檻 0.40rad）'.format(dyaw)})
    else:
        base.update({'ok':True,'type':'ok',
                     'message':('可從 WP{} 接續'.format(next_idx+1) if next_idx>0 else '起點對齊')})
    report(base)

if __name__=='__main__':main()
