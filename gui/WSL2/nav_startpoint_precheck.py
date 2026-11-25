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


def report(payload):
    print('NAV_STARTPOINT_RESULT='+json.dumps(payload,ensure_ascii=True,sort_keys=True))


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--csv',required=True)
    args=p.parse_args(rospy.myargv()[1:])
    try:
        info=validate(load_route(args.csv))
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
    base={'distance_m':round(d,4),'limit_m':0.50,'current_xy':[round(p.x,4),round(p.y,4)],
          'start_xy':[round(sx,4),round(sy,4)],'yaw_difference_rad':round(dyaw,4)}
    if d>0.50:
        base.update({'ok':False,'type':'too_far','message':'距離 Waypoint 起點 {:.2f}m，請先遙控移到起點附近'.format(d)})
    elif dyaw>0.40:
        base.update({'ok':False,'type':'yaw_mismatch','message':'車頭與起始方向相差 {:.2f}rad（門檻 0.40rad）'.format(dyaw)})
    else:
        base.update({'ok':True,'type':'ok','message':'起點對齊'})
    report(base)

if __name__=='__main__':main()
