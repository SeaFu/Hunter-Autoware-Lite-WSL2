#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""READ-ONLY NDT recording feasibility overlay for ROS Melodic.

Subscribes to /ndt_pose and /points_map; publishes ONLY visualization and
informational JSON. No vehicle-control publishers are created.
"""
from __future__ import print_function
import argparse
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from nav_trajectory_feasibility import (sample_trajectory, evaluate_segments,
    profile_id, counts, COLORS)


class FeasibilityRecorder(object):
    def __init__(self, profile, output):
        self.profile=profile
        self.output=output
        self.positions=[]
        self.latest_pose=None
        self.assessment=[]
        self.map_xy=[]
        self.map_ok=False
        self.last_map=0.0
        self.last_sample_time=0.0
        self.last_pub=0.0
        self.last_result=None
        self.last_jump=False
        self.dropped_bad=0
        self.pub=rospy.Publisher('/hunter_nav/feasibility_markers', MarkerArray, queue_size=1, latch=True)
        self.state_pub=rospy.Publisher('/hunter_nav/feasibility_state', String, queue_size=1, latch=True)
        rospy.Subscriber('/points_map',PointCloud2,self.on_map,queue_size=1)
        rospy.Subscriber('/ndt_pose',PoseStamped,self.on_pose,queue_size=50)
        rospy.on_shutdown(self.finalize)
        rospy.logwarn('Feasibility preview READ ONLY, profile %s, lookahead %.2fm',
                     profile_id(profile),max(profile['min_lookahead_m'],profile['speed_mps']*profile['lookahead_ratio']))

    def on_map(self,msg):
        if msg.header.frame_id and msg.header.frame_id.lstrip('/')!='map':
            rospy.logwarn_throttle(10,'Ignore PCD points outside map frame')
            return
        if self.map_ok: return
        # Focus on likely obstacles above floor, not ground returns. Height
        # threshold is an uncalibrated heuristic, never a safety guarantee.
        xy=[]
        unique=set()
        try:
            for x,y,z in pc2.read_points(msg,field_names=('x','y','z'),skip_nans=True):
                if not (.20 <= z <= 1.70): continue
                key=(int(math.floor(x/.16)),int(math.floor(y/.16)))
                if key not in unique:
                    unique.add(key);xy.append((float(x),float(y)))
            self.map_xy=xy
            self.map_ok=bool(xy)
            rospy.logwarn('Map XY height-filtered %d cells (advisory obstacle proxy)',len(xy))
        except Exception as exc:
            rospy.logwarn('Map preview unavailable: %s',exc)

    def on_pose(self,msg):
        p=msg.pose.position
        q=msg.pose.orientation
        yaw=math.atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z))
        if not all(not (math.isnan(v) or math.isinf(v)) for v in (p.x,p.y,p.z,yaw)):
            self.dropped_bad+=1;return
        item={'x':p.x,'y':p.y,'z':p.z,'yaw':yaw}
        self.latest_pose=item
        if self.positions:
            d=math.hypot(p.x-self.positions[-1]['x'],p.y-self.positions[-1]['y'])
            if d>1.50:
                self.dropped_bad+=1;self.last_jump=True
                rospy.logwarn_throttle(3.0,'NDT jump %.2fm: mark preview unverified',d)
                return
            if d < self.profile['spacing_m']:
                return
        self.positions.append(item)
        if len(self.positions)>5000:
            self.dropped_bad+=1;return
        if time.time()-self.last_pub>=.5:
            self.update(False)

    def route_map_subset(self):
        if not self.positions or not self.map_ok:
            return None
        x=[p['x'] for p in self.positions];y=[p['y'] for p in self.positions]
        margin=self.profile['corridor_half_width_m']+1.0
        xmin,xmax=min(x)-margin,max(x)+margin
        ymin,ymax=min(y)-margin,max(y)+margin
        subset=[p for p in self.map_xy if xmin<=p[0]<=xmax and ymin<=p[1]<=ymax]
        return subset

    def update(self, complete=False):
        self.last_pub=time.time()
        self.assessment=evaluate_segments(self.positions,self.profile,self.route_map_subset(),complete)
        self.last_result=dict(schema='hunter_feasibility_v1', profile=self.profile,
            check_profile=profile_id(self.profile), samples=self.positions[:],
            segments=self.assessment[:], map_loaded=self.map_ok,
            jump_detected=self.last_jump, invalid_poses=self.dropped_bad,
            completed=bool(complete))
        if self.last_jump:
            for item in self.assessment[:-1]:
                if item['segment_status']=='PASS':
                    item['segment_status']='WARN'; item['segment_reason']='NDT_POSE_JUMP'
        self.last_result['segments']=self.assessment[:]
        self.write_state()
        self.state_pub.publish(String(json.dumps(dict(check_profile=profile_id(self.profile),
            counts=counts(self.assessment),point_count=len(self.positions),
            map_loaded=self.map_ok,completed=bool(complete)))))
        self.render()

    def write_state(self):
        if not self.output or not self.last_result:return
        tmp=self.output+'.tmp'
        try:
            with open(tmp,'w') as f:
                json.dump(self.last_result,f,separators=(',',':'))
            os.rename(tmp,self.output)
        except (IOError,OSError) as exc:
            rospy.logerr_throttle(10,'Preview state save failed: %s',exc)

    def render(self):
        msg=MarkerArray()
        clear=Marker();clear.header.frame_id='map';clear.action=Marker.DELETEALL
        msg.markers.append(clear)
        for i,item in enumerate(self.assessment[:-1]):
            start,end=self.positions[i],self.positions[i+1]
            m=Marker();m.header.frame_id='map';m.header.stamp=rospy.Time.now()
            m.ns='recorded_segment_status';m.id=i+1;m.type=Marker.LINE_STRIP;m.action=Marker.ADD
            m.pose.orientation.w=1.0;m.scale.x=.085
            r,g,b=COLORS[item['segment_status']]
            m.color.r,m.color.g,m.color.b,m.color.a=r,g,b,1.0
            from geometry_msgs.msg import Point
            for pose in (start,end):
                point=Point();point.x,point.y,point.z=pose['x'],pose['y'],.12
                m.points.append(point)
            msg.markers.append(m)
            if item['segment_status'] in ('FAIL','WARN') and i%2==0:
                t=Marker();t.header=m.header;t.ns='feasibility_warning';t.id=i+1
                t.type=Marker.TEXT_VIEW_FACING;t.action=Marker.ADD;t.pose.orientation.w=1.0
                t.pose.position.x=(start['x']+end['x'])/2.0
                t.pose.position.y=(start['y']+end['y'])/2.0
                t.pose.position.z=.50;t.scale.z=.18
                t.color.r,t.color.g,t.color.b,t.color.a=r,g,b,.95
                t.text='S{:02d} {}'.format(i+1,item['segment_reason'][:19]);msg.markers.append(t)
        if self.positions:
            # Corridor is a visualization of a centerline-offset envelope;
            # this does not model turning swept corners of the full vehicle.
            half=self.profile['corridor_half_width_m']
            for direction,label in ((-1,'left'),(1,'right')):
                m=Marker();m.header.frame_id='map';m.ns='preview_corridor_'+label;m.id=6000
                m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.0
                m.scale.x=.022;m.color.r,m.color.g,m.color.b,m.color.a=.1,.65,1.0,.7
                from geometry_msgs.msg import Point
                for p in self.positions:
                    off=Point();off.x=p['x']-direction*half*math.sin(p['yaw'])
                    off.y=p['y']+direction*half*math.cos(p['yaw']);off.z=.035
                    m.points.append(off)
                msg.markers.append(m)
            q=self.positions[-1];body=Marker();body.header.frame_id='map';body.ns='preview_vehicle_size'
            body.id=6100;body.type=Marker.CUBE;body.action=Marker.ADD;body.pose.orientation.w=math.cos(q['yaw']/2.0)
            body.pose.orientation.z=math.sin(q['yaw']/2.0)
            body.pose.position.x=q['x'];body.pose.position.y=q['y'];body.pose.position.z=.04
            body.scale.x=.75;body.scale.y=self.profile['vehicle_width_m'];body.scale.z=.025
            body.color.r,body.color.g,body.color.b,body.color.a=.15,.7,1.0,.25
            msg.markers.append(body)
            info=Marker();info.header.frame_id='map';info.ns='preview_nav_settings'
            info.id=6200;info.type=Marker.TEXT_VIEW_FACING;info.action=Marker.ADD
            info.pose.orientation.w=1.0
            info.pose.position.x=q['x'];info.pose.position.y=q['y'];info.pose.position.z=.92
            info.scale.z=.25
            info.color.r,info.color.g,info.color.b,info.color.a=1.0,1.0,1.0,.95
            look=max(self.profile['min_lookahead_m'],
                     self.profile['lookahead_ratio']*self.profile['speed_mps'])
            info.text='W={:.2f}m | Half={:.2f}m | Ld={:.2f}m | Stop={:.2f}m'.format(
                self.profile['vehicle_width_m'],half,look,self.profile['obstacle_stop_m'])
            msg.markers.append(info)
        self.pub.publish(msg)

    def finalize(self):
        if self.positions:
            if self.latest_pose is not None:
                d=math.hypot(self.positions[-1]['x']-self.latest_pose['x'],
                             self.positions[-1]['y']-self.latest_pose['y'])
                if .05 <= d <= 1.50:
                    self.positions.append(self.latest_pose)
            self.update(True)
            rospy.logwarn('Feasibility final %d points; status %s',len(self.positions),counts(self.assessment))


def main():
    arg=argparse.ArgumentParser()
    arg.add_argument('--profile-json',required=True)
    arg.add_argument('--state-file',required=True)
    options=arg.parse_args(rospy.myargv()[1:])
    profile=json.loads(options.profile_json)
    rospy.init_node('hunter_trajectory_feasibility_preview',anonymous=False)
    FeasibilityRecorder(profile,options.state_file)
    rospy.spin()

if __name__=='__main__':main()
