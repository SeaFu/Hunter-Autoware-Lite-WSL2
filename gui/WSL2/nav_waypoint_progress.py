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
from nav_ordered_lookahead import local_station


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


def remaining_path_length(points, next_index, x, y):
    """Arc length remaining, projected ONLY onto the current incoming segment.

    A closed loop has near-zero endpoint distance at WP01, but its remaining
    arc is still the entire route. Never project onto a distant future leg.
    """
    if len(points) < 2 or not 0 <= next_index < len(points):
        raise ValueError('Invalid route/progress for remaining length')
    i = max(0, next_index - 1)
    a, b = points[i], points[i + 1]
    dx, dy = b['x'] - a['x'], b['y'] - a['y']
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        raise ValueError('Degenerate route segment')
    along = ((x-a['x'])*dx + (y-a['y'])*dy) / length
    along = max(0.0, min(length, along))
    return length - along + sum(
        math.hypot(q['x']-p['x'], q['y']-p['y'])
        for p, q in zip(points[i+1:], points[i+2:]))


def final_waypoint_reached(points, next_index, x, y, yaw):
    """Completion requires ordered progress AND reaching the final segment end.

    The 0.55m XY envelope is retained only as an additional upper bound.
    The local along-track gate prevents stopping at the start of a short
    final segment, even when both endpoints fit within that envelope.
    """
    if len(points) < 3 or next_index != len(points)-1:
        return False
    a, b = points[-2], points[-1]
    dx, dy = b['x']-a['x'], b['y']-a['y']
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return False
    ex, ey = x-a['x'], y-a['y']
    along = (ex*dx+ey*dy)/length
    lateral = abs(ex*dy-ey*dx)/length
    margin = min(0.06, 0.20*length)
    reach = min(0.30, max(0.08, 0.70*length))
    heading = math.atan2(dy, dx)
    error = abs(math.atan2(math.sin(yaw-heading), math.cos(yaw-heading)))
    return (along >= length-margin and lateral <= reach and error <= 0.70
            and math.hypot(x-b['x'], y-b['y']) <= 0.55)


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

    def gate_reach(self,index):
        # A checkpoint is a local cross-section, not a tiny XY circle.
        # Allow modest lateral tracking error on an Ackermann bend, while
        # remaining tighter than the normal route deviation STOP limit.
        return min(self.reach_m,max(.40,self.local_reach(index)))

    def update(self, x, y, following, yaw=None):
        if not following or self.completed:
            return False
        n=len(self.points)
        old=self.next_index
        # The final WP remains pending until the supervisor has independently
        # checked the final segment and stopped. Only its GOAL acknowledgement
        # may mark this run as complete/reset the persisted index.
        if old>=n-1:
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
            # Match the existing 0.50m START alignment allowance when the
            # first segment is short (e.g. 0.25m). No index may be skipped.
            if along < -0.05 or along > min(0.65,max(0.50,length+0.12)):
                return False
            if lateral > min(0.30,max(0.13,self.local_reach(0))):
                return False
            if yaw is not None:
                h=math.atan2(dy,dx)
                if abs(math.atan2(math.sin(yaw-h),math.cos(yaw-h)))>0.65:
                    return False
            self.next_index=1
            return True
        # Project only onto a short contiguous local sequence, with heading
        # checks for the next segments. This permits crossing a waypoint
        # station with lateral error; it does not require hitting every
        # recorded point's tiny circle. Never search across the whole loop.
        if yaw is None:
            a,b=self.points[old-1],self.points[old]
            yaw=math.atan2(b['y']-a['y'],b['x']-a['x'])
        best=local_station(self.points,old,x,y,yaw)
        if best is None or best[2]>self.gate_reach(old):
            return False
        best_s=self.arc[old-1]+best[3]
        # Advance one waypoint per valid pose, only after passing its local
        # along-route gate. It must not skip WP indices at a sharp corner.
        target_arc=self.arc[old]
        margin=min(0.06,0.20*(self.arc[old]-self.arc[old-1]))
        if best_s<target_arc-margin:
            return False
        self.next_index+=1
        return self.next_index!=old

    def acknowledge_goal(self):
        """Latch completion in memory; disk may reset for the next explicit START."""
        if self.completed or self.next_index != len(self.points)-1:
            return False
        self.completed = True
        return True
