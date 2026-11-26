#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Ordered path-arc target and Pure Pursuit geometry, independent of ROS.
The actual target is not required to be outside a Euclidean lookahead circle.
"""
from __future__ import division
import math

def local_station(points,next_index,x,y,yaw,max_forward_arc=.85,max_segments=4):
    """Heading-aware projection on a SHORT contiguous upcoming route only.

    Returns (segment index, fraction, distance, arc from incoming start).
    Never search the entire loop; prefer earlier segments if almost equally
    close, and only consider forward segments with compatible heading.
    """
    n=len(points)
    if n<2 or not 0<=next_index<n:return None
    start=max(0,next_index-1)
    incoming=math.hypot(points[start+1]['x']-points[start]['x'],
                        points[start+1]['y']-points[start]['y']) if start+1<n else 0.
    budget=incoming+max_forward_arc
    station=0.;best=None
    for i in range(start,min(n-1,start+max_segments)):
        a,b=points[i],points[i+1]
        dx,dy=b['x']-a['x'],b['y']-a['y']
        length=math.hypot(dx,dy)
        if length<=1e-9:continue
        if station>budget:break
        upper=min(1.,max(0.,(budget-station)/length))
        heading=math.atan2(dy,dx)
        error=abs(math.atan2(math.sin(yaw-heading),math.cos(yaw-heading)))
        if error<= (1.15 if i<=start+1 else .85):
            t=max(0.,min(upper,((x-a['x'])*dx+(y-a['y'])*dy)/(length*length)))
            dist=math.hypot(x-a['x']-t*dx,y-a['y']-t*dy)
            candidate=(i,t,dist,station+t*length)
            # Small distance tie favors the earlier station (avoid adjacent
            # crossing/return legs). A clearly closer local segment may win.
            if best is None or dist<best[2]-.025:
                best=candidate
        station+=length
    return best


def ordered_target(points,next_index,x,y,lookahead,yaw=None):
    """Return (segment end index, interpolation fraction, arc distance).
    Start with the incoming segment. With heading supplied, a nearby,
    contiguous forward segment (bounded local window) can anchor the
    preview even if the checkpoint has not been acknowledged yet.
    This does NOT advance the persisted checkpoint or complete the route.
    """
    n=len(points)
    if n<3 or not 0<=next_index<n or lookahead<=0:
        raise ValueError('Invalid ordered lookahead')
    i=max(0,next_index-1)
    a,b=points[i],points[i+1]
    dx,dy=b['x']-a['x'],b['y']-a['y']
    length=math.hypot(dx,dy)
    if length<=1e-9:raise ValueError('Degenerate waypoint')
    t=max(0.,min(1.,((x-a['x'])*dx+(y-a['y'])*dy)/(length*length)))
    if yaw is not None and next_index>0:
        local=local_station(points,next_index,x,y,yaw)
        if local is not None and local[2]<=.40:
            i,t=local[0],local[1]
            a,b=points[i],points[i+1]
            length=math.hypot(b['x']-a['x'],b['y']-a['y'])
    remain=lookahead
    available=length*(1.-t)
    if remain<=available:
        return i+1,t+remain/length,lookahead
    remain-=available
    for j in range(i+1,n-1):
        a,b=points[j],points[j+1]
        length=math.hypot(b['x']-a['x'],b['y']-a['y'])
        if length<=1e-9:continue
        if remain<=length:
            return j+1,remain/length,lookahead
        remain-=length
    return n-1,1.,max(0.,lookahead-remain)


def ordered_pursuit(points,next_index,x,y,yaw,lookahead,speed):
    """Return a target and geometric twist using the ordered path arc.

    The selected target may be closer in Euclidean distance than the arc
    length, e.g. at a corner or just before the final waypoint. That does not
    invalidate Pure Pursuit. A target behind the vehicle or a geometrically
    degenerate target is not followed forwards. Only the supervisor decides
    real motion, clearance, speed limits, recovery and completion.
    """
    values=(x,y,yaw,lookahead,speed)
    if not all(not(math.isnan(v) or math.isinf(v)) for v in values):
        raise ValueError('Non-finite Pure Pursuit input')
    if speed<0:
        raise ValueError('Ordered forward pursuit requires nonnegative speed')
    end,t,arc=ordered_target(points,next_index,x,y,lookahead,yaw=yaw)
    a,b=points[end-1],points[end]
    tx=a['x']+(b['x']-a['x'])*t
    ty=a['y']+(b['y']-a['y'])*t
    dx,dy=tx-x,ty-y
    fx=math.cos(yaw)*dx+math.sin(yaw)*dy
    fy=-math.sin(yaw)*dx+math.cos(yaw)*dy
    d2=dx*dx+dy*dy
    # No forward movement toward a target behind the car. Heading/obstacle
    # recovery remains a separate, guarded supervisor state.
    if d2<1e-8 or fx<=0.0:
        return dict(end=end,arc=arc,target=(tx,ty),distance=math.sqrt(d2),
                    kappa=0.,v=0.,w=0.,reason='target_not_ahead')
    k=2.*fy/d2
    return dict(end=end,arc=arc,target=(tx,ty),distance=math.sqrt(d2),
                kappa=k,v=speed,w=speed*k,reason='ok')
