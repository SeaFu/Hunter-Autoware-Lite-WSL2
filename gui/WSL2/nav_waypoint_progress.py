#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Planar monotonic waypoint progress; works in ROS Melodic Python2 and GUI Python3.

A progress file is tied to the actual CSV contents. It is never advanced while
manual driving or while the navigation supervisor is paused/stopped.
"""
from __future__ import division
import hashlib
import json
import math
import os


def fingerprint(points):
    data = [[round(p['x'], 5), round(p['y'], 5), round(p['yaw'], 5)] for p in points]
    return hashlib.sha256(json.dumps(data, separators=(',', ':')).encode('utf-8')).hexdigest()


def progress_path(csv_path):
    return os.path.abspath(csv_path) + '.progress.json'


def read_progress(csv_path, points):
    path = progress_path(csv_path)
    try:
        with open(path, 'r') as stream:
            payload = json.load(stream)
        if payload.get('fingerprint') != fingerprint(points):
            return 0
        idx = int(payload['next_index'])
        return idx if 0 <= idx < len(points) else 0
    except (IOError, OSError, ValueError, TypeError, KeyError):
        return 0


def write_progress(csv_path, points, next_index):
    path = progress_path(csv_path)
    data = {'fingerprint': fingerprint(points), 'next_index': int(next_index),
            'count': len(points), 'format': 1}
    tmp = path + '.tmp'
    with open(tmp, 'w') as stream:
        json.dump(data, stream, sort_keys=True)
    os.rename(tmp, path)  # POSIX atomic replacement; program executes in WSL2


def reset_progress(csv_path, points):
    write_progress(csv_path, points, 0)


def distance_to_remaining(points, next_index, x, y, max_forward_segments=None):
    """Nearest distance + segment heading within not-yet-traversed route.

    Include the last completed waypoint as an anchor, but not any earlier
    segment. This prevents automatic rewind to start after an interruption.
    """
    if not points:
        return (float('inf'), 0.0)
    start = max(0, min(int(next_index) - 1, len(points) - 2))
    best = float('inf')
    heading = 0.0
    stop = len(points) - 1 if max_forward_segments is None else min(
        len(points) - 1, start + max(1, int(max_forward_segments)))
    for i in range(start, stop):
        a, b = points[i], points[i + 1]
        vx, vy = b['x'] - a['x'], b['y'] - a['y']
        d2 = vx * vx + vy * vy
        if d2 < 1e-9:
            continue
        t = max(0.0, min(1.0, ((x-a['x'])*vx + (y-a['y'])*vy)/d2))
        d = math.hypot(x-a['x']-t*vx, y-a['y']-t*vy)
        if d < best:
            best = d
            heading = math.atan2(vy, vx)
    return best, heading


class ProgressTracker(object):
    """Track cumulative XY distance along the locally nearest *future* segment.

    This is more reliable than a fixed waypoint-radius test, especially when
    recorded waypoints are spaced only 0.5 m apart.
    """
    def __init__(self, points, initial_index=0, reach_m=0.52):
        if len(points) < 3:
            raise ValueError('Route must contain at least 3 points')
        self.points = points
        self.next_index = max(0, min(int(initial_index), len(points) - 1))
        self.reach_m = float(reach_m)
        self.completed = False
        self.arc = [0.0]
        for a,b in zip(points,points[1:]):
            self.arc.append(self.arc[-1]+math.hypot(b['x']-a['x'],b['y']-a['y']))

    def local_reach(self, index):
        """A conservative lateral tolerance based on nearby waypoint spacing.

        The old 0.52m constant can overlap several waypoints on a 0.2-0.3m
        route, especially at a U-turn. Keep a spacing-based cap while retaining
        the original reach_m parameter as an additional upper bound.
        """
        n=len(self.arc)
        idx=max(0,min(int(index),n-1))
        distances=[]
        if idx>0:
            distances.append(self.arc[idx]-self.arc[idx-1])
        if idx<n-1:
            distances.append(self.arc[idx+1]-self.arc[idx])
        spacing=min(distances) if distances else self.reach_m
        return min(self.reach_m,0.30,max(0.08,0.70*spacing))

    def update(self, x, y, following, yaw=None):
        if not following or self.completed:
            return False
        n=len(self.points)
        old=self.next_index
        if old>=n:
            return False
        # The GUI permits a starting position roughly 0.30 m from WP01.
        # Requiring the robot to enter a smaller *circle* around WP01 after
        # AUTO becomes ACTIVE deadlocks progress if it has ALREADY passed WP01:
        # the robot only moves farther from that circle.  Use a short, directed
        # projection onto WP01->WP02 instead, with tight lateral/yaw limits.
        # This admits positions slightly ahead of WP01 without skipping a
        # distant segment or accidentally matching the opposite U-turn leg.
        if old==0:
            a,b=self.points[0],self.points[1]
            dx,dy=b['x']-a['x'],b['y']-a['y']
            length=math.hypot(dx,dy)
            if length<=1e-6:
                return False
            ex,ey=x-a['x'],y-a['y']
            along=(ex*dx+ey*dy)/length
            lateral=abs(ex*dy-ey*dx)/length
            # Allow a modest amount beyond WP02 to handle delayed first ACTIVE
            # after the starting alignment check, but not a remote waypoint.
            if along < -0.05 or along > min(0.65,length+0.12):
                return False
            if lateral > min(0.30,max(0.13,self.local_reach(0))):
                return False
            if yaw is not None:
                h=math.atan2(dy,dx)
                if abs(math.atan2(math.sin(yaw-h),math.cos(yaw-h)))>0.65:
                    return False
            self.next_index=1
            return True
        # Only inspect the immediately preceding and current segments. The
        # previous implementation searched four FUTURE segments and allowed up
        # to three waypoint advances from a single pose, which can jump across
        # tightly folded lanes.
        begin=max(0,old-1)
        end=min(n-1,old+1)
        best_d,best_s=float('inf'),None
        for i in range(begin,end):
            a,b=self.points[i],self.points[i+1]
            dx,dy=b['x']-a['x'],b['y']-a['y']
            d2=dx*dx+dy*dy
            if d2<=1e-9:
                continue
            seg_heading=math.atan2(dy,dx)
            if yaw is not None:
                heading_error=math.atan2(math.sin(yaw-seg_heading), math.cos(yaw-seg_heading))
                if abs(heading_error)>1.15:
                    continue
            t=max(0.0,min(1.0,((x-a['x'])*dx+(y-a['y'])*dy)/d2))
            lateral=math.hypot(x-a['x']-t*dx,y-a['y']-t*dy)
            arc=self.arc[i]+t*math.sqrt(d2)
            if lateral<best_d:
                best_d,best_s=lateral,arc
        if best_s is None or best_d>self.local_reach(old):
            return False
        # Advance one waypoint per valid pose, only after passing its local
        # along-route gate. It must not skip WP indices at a sharp corner.
        target_arc=self.arc[old]
        margin=min(0.06,0.20*(self.arc[old]-self.arc[old-1]))
        if best_s<target_arc-margin:
            return False
        self.next_index+=1
        if self.next_index>=n:
            if math.hypot(x-self.points[-1]['x'],y-self.points[-1]['y'])<=0.55:
                self.next_index=0
                self.completed=True
            else:
                self.next_index=n-1
        return self.next_index!=old or self.completed
