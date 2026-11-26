#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Pure Python route checks. Shared by Python3 GUI and Python2 ROS nodes.

The route is in NDT 'map' coordinates; velocity CSV field is km/h.
Planar XY/yaw route check: Z height differences do NOT block validation.
This does not validate NDT/TF consistency or prove safe terrain.
This is an engineering screening check, not an assurance of safe traversability.
"""
from __future__ import division
import csv
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_route(path):
    points = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        required = ('x','y','z','yaw','velocity','change_flag')
        if not reader.fieldnames or not set(required).issubset(set(reader.fieldnames)):
            raise ValueError('Waypoint CSV 必須含 x,y,z,yaw,velocity,change_flag')
        for n, row in enumerate(reader, 2):
            try:
                item = dict((key, float(row[key])) for key in required)
            except (TypeError, ValueError, KeyError):
                raise ValueError('第 {} 行有無效數值'.format(n))
            if not all((not math.isnan(value) and not math.isinf(value)) for value in item.values()):
                raise ValueError('第 {} 行存在 NaN/Inf'.format(n))
            # Extended feasibility columns are optional; old 6-column CSV
            # remains valid, and numerical navigation validation is unchanged.
            if 'segment_status' in row:
                st=(row.get('segment_status') or 'PENDING').strip().upper()
                item['segment_status']=st if st in ('PASS','WARN','FAIL','PENDING') else 'PENDING'
                item['segment_reason']=(row.get('segment_reason') or '').strip()
                item['check_profile']=(row.get('check_profile') or '').strip()
                for field in ('min_radius_m','predicted_error_m','clearance_m'):
                    try:item[field]=float(row[field]) if row.get(field,'').strip() else None
                    except (ValueError,TypeError):item[field]=None
            points.append(item)
    return points


def validate(points):
    """Screen route geometry in XY only; preserve input Z for display/ROS data.

    No Z-step or slope rejection. Retain path spacing, yaw, speed,
    stop-at-goal and starting heading guards.
    """
    if len(points) < 5:
        raise ValueError('實車測試需要至少 5 個 Waypoint')
    if len(points) > 5000:
        raise ValueError('Waypoint 過多，請拆分為較短的路徑')
    length, max_step = 0., 0.
    for index, point in enumerate(points):
        if point['velocity'] < 0 or point['velocity'] > 5.0:
            raise ValueError('第 {} 點 velocity 超出 0~5 km/h'.format(index + 1))
        if point['velocity'] <= 0 and index < len(points) - 1:
            raise ValueError('僅允許終點 velocity=0，請修正中途停車點')
        if index:
            prev = points[index-1]
            dxy = math.hypot(point['x']-prev['x'], point['y']-prev['y'])
            if dxy < 0.05:
                raise ValueError('第 {}~{} 點 XY 過近，可能有重複或定位漂移'.format(index, index+1))
            if dxy > 1.50:
                raise ValueError('第 {}~{} 點間距 {:.2f}m > 1.5m'.format(index, index+1, dxy))
            if abs(wrap(point['yaw']-prev['yaw'])) > 0.70:
                raise ValueError('第 {}~{} 點航向突變，請重新檢查路徑'.format(index,index+1))
            length += dxy
            max_step = max(max_step,dxy)
    if length < 2.0:
        raise ValueError('路徑長度不足 2 m，無法驗證追蹤')
    if points[-1]['velocity'] > 0.0001:
        raise ValueError('最後一個 Waypoint 必須為 0 km/h（停車點）')
    a,b = points[0],points[1]
    heading = math.atan2(b['y']-a['y'],b['x']-a['x'])
    if abs(wrap(a['yaw']-heading)) > 0.55:
        raise ValueError('起始 yaw 與路徑方向差異過大，可能不適合正向行駛')
    return {'count':len(points),'length_m':length,'max_gap_m':max_step,'start':(a['x'],a['y'],a['yaw']),
            'goal':(points[-1]['x'],points[-1]['y'])}


def nearest_route_distance(points,x,y):
    """Nearest distance to polyline, not only waypoint centers."""
    best=float('inf')
    for p,q in zip(points,points[1:]):
        vx=q['x']-p['x']; vy=q['y']-p['y']
        t=((x-p['x'])*vx+(y-p['y'])*vy)/(vx*vx+vy*vy)
        t=max(0.0,min(1.0,t))
        d=math.hypot(x-p['x']-t*vx,y-p['y']-t*vy)
        if d<best:best=d
    return best


def starting_alignment(info,x,y,yaw,dist_limit=0.50,yaw_limit=0.40):
    sx,sy,sh=info['start']
    d=math.hypot(x-sx,y-sy)
    dyaw=abs(wrap(yaw-sh))
    if d>dist_limit:
        return False,'與路徑起點相距 {:.2f}m > {:.2f}m；請遙控回到起點'.format(d,dist_limit)
    if dyaw>yaw_limit:
        return False,'車頭與路徑起點 yaw 差 {:.2f}rad > {:.2f}rad'.format(dyaw,yaw_limit)
    return True,'起點距離 {:.2f}m，航向誤差 {:.2f}rad'.format(d,dyaw)


def nearest_route_heading(points,x,y):
    """Heading of closest route segment in XY, for wrong-way detection."""
    best=float('inf')
    heading=0.0
    for p,q in zip(points,points[1:]):
        vx=q['x']-p['x'];vy=q['y']-p['y']
        t=((x-p['x'])*vx+(y-p['y'])*vy)/(vx*vx+vy*vy)
        t=max(0.0,min(1.0,t))
        d=math.hypot(x-p['x']-t*vx,y-p['y']-t*vy)
        if d<best:
            best=d;heading=math.atan2(vy,vx)
    return heading
