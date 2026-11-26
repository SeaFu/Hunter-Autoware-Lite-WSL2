#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Shared, NON-ACTUATING trajectory feasibility preview (Python 2 + 3).

The preview simulates Pure Pursuit tracking, NOT certified collision checking.
No hardcoded minimum-turn-radius failure or curvature clipping is used.
Segments are row[i] -> row[i+1]. A point's status is assigned to its outgoing
segment. Keep this module pure so live ROS2/Python2 and offline CSV/Python3
use the exact same assessment logic and thresholds.
"""
from __future__ import division
import hashlib
import json
import math

STATUS = ('PASS', 'WARN', 'FAIL', 'PENDING')
COLORS = {
    'PASS': (0.12, 0.93, 0.36), 'WARN': (1.0, 0.75, 0.16),
    'FAIL': (1.0, 0.18, 0.20), 'PENDING': (0.60, 0.65, 0.72),
}
EXTRA_FIELDS = ('segment_status', 'segment_reason', 'predicted_error_m',
                'predicted_progress_m', 'clearance_m', 'check_profile')


def as_points(samples):
    return [dict(x=float(p['x']), y=float(p['y']), z=float(p.get('z', 0.0)),
                 yaw=float(p.get('yaw', 0.0))) for p in samples]


def sample_trajectory(samples, spacing):
    """Match generate_waypoints: accepted pose must be spaced from last accepted."""
    if spacing < .05 or math.isnan(spacing) or math.isinf(spacing):
        raise ValueError('invalid spacing')
    if not samples:
        return []
    out = [samples[0]]
    for item in samples[1:]:
        dx = float(item['x']) - float(out[-1]['x'])
        dy = float(item['y']) - float(out[-1]['y'])
        gap = math.hypot(dx, dy)
        if gap >= spacing:
            out.append(item)
    return out


def profile_for(ratio, lookahead_min, vehicle_width, corridor_half,
                deviation_limit, obstacle_stop, speed_mps, min_radius=None,
                spacing=.25, nominal_radius=None):
    """The historic radius kwargs are ignored for old caller compatibility.

    A manual-drive route is not rejected using an assumed vehicle radius.
    Control saturation used in tracking simulation is *angular rate and
    acceleration*, not a fixed minimum turning radius.
    """
    result = dict(lookahead_ratio=float(ratio), min_lookahead_m=float(lookahead_min),
                  vehicle_width_m=float(vehicle_width), corridor_half_width_m=float(corridor_half),
                  deviation_limit_m=float(deviation_limit), obstacle_stop_m=float(obstacle_stop),
                  speed_mps=float(speed_mps), spacing_m=float(spacing),
                  evaluator='pp_tracking_preview_v1.6.3')
    if not (.05 <= result['spacing_m'] <= 5.0 and
            result['corridor_half_width_m'] >= result['vehicle_width_m'] / 2.0 and
            .05 < result['speed_mps'] <= 5.0 and
            result['lookahead_ratio'] > 0 and result['min_lookahead_m'] > 0 and
            result['deviation_limit_m'] > 0):
        raise ValueError('invalid Pure Pursuit tracking preview profile')
    return result


def profile_id(profile):
    blob = json.dumps(profile, sort_keys=True, separators=(',', ':')).encode('utf-8')
    return hashlib.sha256(blob).hexdigest()[:16]


def distance_xy(a, b):
    return math.hypot(float(a['x']) - float(b['x']), float(a['y']) - float(b['y']))


def _projection_dist(x, y, a, b):
    vx, vy = b['x']-a['x'], b['y']-a['y']
    length = vx*vx+vy*vy
    if length < 1e-10:
        return math.hypot(x-a['x'],y-a['y'])
    t = max(0.0,min(1.0,((x-a['x'])*vx+(y-a['y'])*vy)/length))
    return math.hypot(x-a['x']-t*vx, y-a['y']-t*vy)


def _arc_lengths(points):
    s=[0.0]
    for a,b in zip(points,points[1:]):
        s.append(s[-1]+distance_xy(a,b))
    return s


def _point_on_path(points, arcs, s):
    s=max(0.0,min(float(s),arcs[-1]))
    for j in range(len(points)-1):
        if arcs[j+1]>=s:
            gap=arcs[j+1]-arcs[j]
            t=(s-arcs[j])/gap if gap>1.e-7 else 0.0
            a,b=points[j],points[j+1]
            return a['x']+(b['x']-a['x'])*t,a['y']+(b['y']-a['y'])*t
    return points[-1]['x'],points[-1]['y']


def _local_progress(points, arcs, x, y, old_s, max_advance, window):
    """Forward-only local polyline projection to avoid jumping across U-turn legs."""
    best=None
    hi=min(arcs[-1],old_s+max_advance)
    lo=max(0.0,old_s-.10)
    for j,(a,b) in enumerate(zip(points,points[1:])):
        if arcs[j+1]<lo or arcs[j]>hi:
            continue
        vx,vy=b['x']-a['x'],b['y']-a['y']
        length2=vx*vx+vy*vy
        if length2<1.e-10:continue
        t=max(0.0,min(1.0,((x-a['x'])*vx+(y-a['y'])*vy)/length2))
        proj_s=max(lo,min(hi,arcs[j]+t*(arcs[j+1]-arcs[j])))
        if proj_s<old_s-.02:continue
        px,py=_point_on_path(points,arcs,proj_s)
        error=math.hypot(x-px,y-py)
        if best is None or error<best[0]:best=(error,proj_s)
    if best is None:return old_s
    return max(old_s,best[1])


def predict_tracking(points, idx, lookahead, speed_mps):
    """Unactuated, local Pure Pursuit kinematic replay (research estimate).

    Lookahead is on the forward route arc (not the nearest U-turn leg).
    It starts from the *recorded* yaw. The same angular-speed (0.50 rad/s)
    and lateral-acceleration (0.80 m/s^2) soft limits as Supervisor slow
    the simulated speed but do not impose a minimum radius or hard fail.
    This is NOT a vehicle/steering actuator model or a collision certificate.
    """
    if idx>=len(points)-1:
        return dict(error=0.0,progress=0.0,travelled=0.0)
    arcs=_arc_lengths(points)
    initial=arcs[idx]
    available=arcs[-1]-initial
    horizon=min(max(.0,lookahead+.60),max(.0,available-.03))
    if horizon<.05:
        return dict(error=0.0,progress=0.0,travelled=0.0)
    x,y=points[idx]['x'],points[idx]['y']
    yaw=float(points[idx]['yaw'])
    progress=initial
    travelled=0.0
    max_err=0.0
    step_target=min(130,max(8,int(math.ceil(horizon/.04))))
    requested_speed=max(.05,float(speed_mps))
    for _ in range(step_target):
        ds=min(horizon-travelled,max(.025,min(.09,requested_speed*.25)))
        if ds<=.000001:break
        # The lookahead waypoint is chosen from monotonic path progress.
        target=_point_on_path(points,arcs,min(arcs[-1],progress+lookahead))
        dx,dy=target[0]-x,target[1]-y
        dist=max(.08,math.hypot(dx,dy))
        alpha=math.atan2(math.sin(math.atan2(dy,dx)-yaw),
                         math.cos(math.atan2(dy,dx)-yaw))
        kappa=2.0*math.sin(alpha)/dist
        # The actual supervisor slows on curvature; there is NO minimum-radius
        # threshold and no arbitrary clipping at 1/1.66m here.
        eff_v=requested_speed
        if abs(kappa)>1.e-6:
            eff_v=min(eff_v,math.sqrt(.80/abs(kappa)),.50/abs(kappa))
        eff_v=max(.02,eff_v)
        yaw_mid=yaw+.5*kappa*ds
        x+=ds*math.cos(yaw_mid)
        y+=ds*math.sin(yaw_mid)
        yaw=math.atan2(math.sin(yaw+kappa*ds),math.cos(yaw+kappa*ds))
        travelled+=ds
        progress=_local_progress(points,arcs,x,y,progress,ds*1.60+.05,lookahead)
        # Distance to local intended segments, not a folded U-turn's other leg.
        closest=float('inf')
        for j,(a,b) in enumerate(zip(points,points[1:])):
            if (arcs[j+1]<max(initial,progress-max(.4,lookahead*.75)) or
                arcs[j]>min(arcs[-1],progress+max(.4,lookahead*.75))):
                continue
            closest=min(closest,_projection_dist(x,y,a,b))
        if closest!=float('inf'):
            max_err=max(max_err,closest)
        if travelled>=horizon-1.e-6:break
    return dict(error=max_err,progress=progress-initial,travelled=travelled)

def _future_distance(points,index):
    return sum(distance_xy(points[j],points[j+1]) for j in range(index,len(points)-1))


def evaluate_segments(points, profile, map_xy=None, complete=False):
    """Evaluate PP tracking, yaw/spacing and advisory map proximity.

    No minimum turning-radius assumption is made. Dynamic obstacles are still
    handled by the independent *real-time* Supervisor, not these saved colors.
    During recording, leave the last lookahead+0.25m PENDING until the future
    is available. The completed endpoint lacks a forward target and is WARN.
    """
    points=as_points(points)
    half=float(profile['corridor_half_width_m'])
    lookahead=max(float(profile['min_lookahead_m']),
                  float(profile['speed_mps'])*float(profile['lookahead_ratio']))
    speed=float(profile['speed_mps'])
    max_dev=float(profile['deviation_limit_m'])
    has_map=(map_xy is not None and len(map_xy)>0)
    result=[]
    for i in range(len(points)):
        item=dict(segment_status='PENDING',segment_reason='NO_NEXT_POINT',
                  predicted_error_m='',predicted_progress_m='',clearance_m='')
        if i==len(points)-1:
            item['segment_reason']='FINAL_STOP_POINT'
            result.append(item);continue
        available=_future_distance(points,i)
        if len(points)<3 or (not complete and available<lookahead+.25):
            item['segment_reason']='WAIT_LOOKAHEAD'
            result.append(item);continue
        d=distance_xy(points[i],points[i+1])
        tangent=math.atan2(points[i+1]['y']-points[i]['y'],
                           points[i+1]['x']-points[i]['x'])
        heading_error=abs(math.atan2(math.sin(points[i]['yaw']-tangent),
                                     math.cos(points[i]['yaw']-tangent)))
        prediction=predict_tracking(points,i,lookahead,speed)
        pred=prediction['error']
        item['predicted_error_m']=round(pred,3)
        item['predicted_progress_m']=round(prediction['progress'],3)
        # No radius checks: the recorded route was physically driven by Hunter.
        # The question is whether these Lookahead/velocity params can follow it.
        if d<.05 or d>1.5:
            item.update(segment_status='FAIL',segment_reason='WAYPOINT_SPACING')
        elif heading_error>.75:
            item.update(segment_status='FAIL',segment_reason='WAYPOINT_HEADING')
        elif pred>max_dev:
            item.update(segment_status='FAIL',segment_reason='PP_TRACKING_DEVIATION')
        elif (prediction['travelled']>.5 and
              prediction['progress']<.30*prediction['travelled']):
            item.update(segment_status='FAIL',segment_reason='PP_TRACKING_STALL')
        elif (heading_error>.35 or pred>min(.35,max(.12,max_dev*.50)) or
              (prediction['travelled']>.5 and
               prediction['progress']<.65*prediction['travelled'])):
            item.update(segment_status='WARN',segment_reason='PP_TRACKING_MARGIN')
        else:
            item.update(segment_status='PASS',segment_reason='PP_TRACKING_PRECHECK')
        # Advisory static-map clearance, not full vehicle swept-volume proof.
        if has_map:
            a,b=points[i],points[i+1]
            margin=min(_projection_dist(px,py,a,b) for px,py in map_xy)
            item['clearance_m']=round(margin,3) if not math.isinf(margin) else ''
            if margin<half:
                if item['segment_status']=='PASS':item['segment_status']='WARN'
                item['segment_reason']='MAP_RETURN_NEAR_CORRIDOR'
        elif item['segment_status']=='PASS':
            item['segment_status']='WARN'
            item['segment_reason']='NO_MAP_CLEARANCE_CHECK'
        if complete and available<lookahead+.25 and item['segment_status']=='PASS':
            item['segment_status']='WARN';item['segment_reason']='END_SHORT_LOOKAHEAD'
        result.append(item)
    return result


def counts(statuses):
    return dict((s,sum(1 for r in statuses[:-1] if r['segment_status']==s)) for s in STATUS)
