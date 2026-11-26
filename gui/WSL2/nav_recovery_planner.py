#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Conservative, ROS-free local recovery geometry. Never publishes commands.

A planar ray grid is only an engineering aid, not safety-rated perception:
missing returns are UNKNOWN, not clear; independent rear/side coverage and
physical emergency stop must be verified before enabling real recovery.
"""
from __future__ import division
import math
from nav_ordered_lookahead import ordered_target

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def finite(*values):
    return all(not (math.isnan(float(v)) or math.isinf(float(v))) for v in values)

class RecoveryConfig(object):
    def __init__(self, width, front, rear, axle, radius, distance, speed, attempts, margin):
        vals=(width,front,rear,axle,radius,distance,speed,attempts,margin)
        if not finite(*vals) or not (.60<=width<=2.50 and .20<=front<=2.00 and
                .20<=rear<=2.00 and 0<=axle<=rear and .50<=radius<=5.00 and
                .20<=distance<=.80 and .05<=speed<=.15 and int(attempts)==attempts and
                1<=attempts<=3 and .20<=margin<=.60):
            raise ValueError('Invalid measured recovery geometry/limits')
        self.width,self.front,self.rear,self.axle,self.radius=map(float,vals[:5])
        self.distance,self.speed,self.attempts,self.margin=float(distance),float(speed),int(attempts),float(margin)

def transform(pose, x, y):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return pose[0]+c*x-s*y,pose[1]+s*x+c*y

def relative(frame, pose):
    dx,dy=pose[0]-frame[0],pose[1]-frame[1]
    c,s=math.cos(frame[2]),math.sin(frame[2])
    return (c*dx+s*dy,-s*dx+c*dy,wrap(pose[2]-frame[2]))

def arc_pose(start, signed_distance, curvature, axle):
    """Kinematic bicycle arc, radius measured at rear axle; return base pose."""
    x,y,h=start
    a=signed_distance*curvature
    if abs(curvature)<1e-8:
        dx,dy=signed_distance,0.
    else:
        dx,dy=math.sin(a)/curvature,(1-math.cos(a))/curvature
    # base_link is axle metres ahead of the rear axle.
    dx+=axle*(math.cos(a)-1.)
    dy+=axle*math.sin(a)
    x,y=transform(start,dx,dy)
    return x,y,wrap(h+a)

def arc(start, signed_distance, curvature, axle, step=.04):
    n=max(1,int(math.ceil(abs(signed_distance)/step)))
    return [(abs(signed_distance)*i/n,arc_pose(start,signed_distance*i/n,curvature,axle))
            for i in range(n+1)]

def local_projection(route, pose):
    best=None
    for a,b in zip(route,route[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        den=dx*dx+dy*dy
        if den<1e-9:continue
        u=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/den))
        d=math.hypot(pose[0]-a[0]-u*dx,pose[1]-a[1]-u*dy)
        if best is None or d<best[0]:
            best=(d,abs(wrap(pose[2]-math.atan2(dy,dx))))
    return best if best is not None else (float('inf'),float('inf'))

def turn_ahead(points,idx,requested_curvature):
    """Do not perform automatic escape for an arbitrary straight blockage."""
    if idx<=0 or idx>=len(points)-1:
        return False
    headings=[]; length=0.
    for a,b in zip(points[idx-1:],points[idx:]):
        dx,dy=b['x']-a['x'],b['y']-a['y']
        length+=math.hypot(dx,dy)
        headings.append(math.atan2(dy,dx))
        if length>=2.0:break
    return abs(requested_curvature)>=.18 or bool(headings and
                  max(abs(wrap(h-headings[0])) for h in headings)>=.35)

class RayGrid(object):
    """Bounded 2-D observed/occupied grid; hit cells always override free."""
    def __init__(self, points, origin, config, resolution=.10, extent=5.):
        self.res=float(resolution); self.extent=float(extent)
        self.free=set(); self.hits=set()
        self.cfg=config
        if not finite(*origin) or not (0.05<=origin[2]<=1.65):
            return
        # Only rays ending in the obstacle-height band are used to assert
        # free XY cells; returns elsewhere never manufacture free space.
        endpoints=set()
        for p in points:
            x,y,z=p
            if not finite(x,y,z) or not .05<=z<=1.65:
                continue
            if -config.rear<x<config.front and abs(y)<config.width/2.:
                continue # physical body/self returns only
            if abs(x)<=extent and abs(y)<=extent:
                self.hits.add(self.cell(x,y))
                endpoints.add(self.cell(x,y))
            else:
                # A genuine distant return supplies ray evidence up to the
                # bounded grid edge, but is NOT a synthetic obstacle there.
                dx,dy=x-origin[0],y-origin[1]
                t=1.
                for o,d in ((origin[0],dx),(origin[1],dy)):
                    if abs(d)>1e-9:
                        edge=(extent-self.res) if d>0 else (-extent+self.res)
                        t=min(t,(edge-o)/d)
                if 0<t<1:
                    endpoints.add(self.cell(origin[0]+t*dx,origin[1]+t*dy))
        a=self.cell(origin[0],origin[1])
        for b in endpoints:
            for c in self._ray(a,b):
                if c in self.hits:break
                self.free.add(c)
        self.free.difference_update(self.hits)

    def cell(self,x,y):
        return int(math.floor(x/self.res)),int(math.floor(y/self.res))

    def center(self,cell):
        return (cell[0]+.5)*self.res,(cell[1]+.5)*self.res

    def _ray(self,a,b):
        x,y=a; tx,ty=b
        dx=abs(tx-x); dy=-abs(ty-y)
        sx=1 if x<tx else -1; sy=1 if y<ty else -1
        err=dx+dy
        while True:
            yield x,y
            if x==tx and y==ty:return
            e=2*err
            if e>=dy:err+=dy;x+=sx
            if e<=dx:err+=dx;y+=sy

    def clear_pose(self,pose,current,extra=.0):
        """Conservative inflated rectangle, including corners/swing.
        Unknown is allowed ONLY inside the already occupied physical body at
        the current pose, not in an expanded or newly travelled area.
        """
        cfg=self.cfg
        m=cfg.margin+extra+self.res*math.sqrt(2)/2.
        f,r,w=cfg.front+m,cfg.rear+m,cfg.width/2.+m
        radius=math.hypot(max(f,r),w)
        lo=self.cell(pose[0]-radius,pose[1]-radius)
        hi=self.cell(pose[0]+radius,pose[1]+radius)
        for i in range(lo[0],hi[0]+1):
            for j in range(lo[1],hi[1]+1):
                cell=(i,j); x,y=self.center(cell)
                px,py,_=relative(pose,(x,y,0.))
                if not (-r<=px<=f and abs(py)<=w):continue
                if cell in self.hits:return False
                if cell not in self.free:
                    cx,cy,_=relative(current,(x,y,0.))
                    # Require unknown cell fully inside the physical body.
                    half=self.res*math.sqrt(2)/2.
                    if not (-cfg.rear+half<=cx<=cfg.front-half and abs(cy)<=cfg.width/2.-half):
                        return False
        return True

    def clear_arc(self,poses,current,extra=.04):
        return all(self.clear_pose(p,current,extra) for _,p in poses)

    def forward_room(self,pose):
        """Distance to observed hits in the forward body-width strip."""
        best=self.extent
        for cell in self.hits:
            x,y=self.center(cell); px,py,_=relative(pose,(x,y,0.))
            if px>self.cfg.front and abs(py)<=self.cfg.width/2.+self.cfg.margin:
                best=min(best,px-self.cfg.front)
        return best

def local_route(points,index,frame,max_length=3.):
    result=[]; length=0.
    for p in points[max(0,index-1):]:
        x,y,_=relative(frame,(p['x'],p['y'],0.))
        if result:length+=math.hypot(x-result[-1][0],y-result[-1][1])
        result.append((x,y))
        if length>=max_length:break
    return result

def forward_candidate(grid,pose,route,current):
    # Use the SAME along-route preview rule as the actual bridge, not the
    # first arbitrary point 0.45m away. Tight corners otherwise rejected every
    # reverse candidate even in an empty scene because their rejoin target
    # required an impossible curvature. Evaluate the physically bounded arc.
    if len(route)<3:return 0.
    points=[{'x':x,'y':y} for x,y in route]
    lookahead=getattr(grid.cfg,'lookahead',2.)
    i,t,_=ordered_target(points,1,pose[0],pose[1],lookahead,yaw=pose[2])
    a,b=route[i-1],route[i]
    px,py,_=relative(pose,(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),0.))
    if px<=.05 or px*px+py*py<1e-8:return 0.
    requested=2.*py/(px*px+py*py)
    cap=getattr(grid.cfg,'forward_curvature',1./grid.cfg.radius)
    k=max(-cap,min(cap,requested))
    safe=0.
    for d in (.25,.50,.75,1.):
        if not grid.clear_arc(arc(pose,d,k,grid.cfg.axle),current):
            break
        safe=d
    return safe

def plan_reverse(grid,current,route,max_cross_track,remaining_budget=None):
    """Evaluate straight/left/right reverse arcs and predicted forward rejoin.
    Returns selected candidate and all candidates for RViz; no safe = None.
    """
    cfg=grid.cfg
    limit=min(cfg.distance,remaining_budget if remaining_budget is not None else cfg.distance)
    if limit<.15:return None,[]
    base_room=grid.forward_room(current)
    base_forward=forward_candidate(grid,current,route,current)
    base_dev,base_heading=local_projection(route,current)
    candidates=[]
    for d in sorted(set([round(limit*.5,3),limit])):
        for k in (0.,-1./cfg.radius,1./cfg.radius):
            poses=arc(current,-d,k,cfg.axle)
            item={'distance':d,'curvature':k,'poses':poses,'ok':False,'score':-1e9,'reason':'collision/unknown'}
            candidates.append(item)
            if not grid.clear_arc(poses,current,extra=.07):continue
            end=poses[-1][1]
            dev,herr=local_projection(route,end)
            if dev>min(.50,max_cross_track) or herr>.60:
                item['reason']='cannot rejoin';continue
            forward=forward_candidate(grid,end,route,current)
            room=grid.forward_room(end)
            if forward<.50 or room<base_room-.05:
                item['reason']='no forward clearance';continue
            if not (room>=base_room+.12 or forward>=base_forward+.25 or herr<=base_heading-.12):
                item['reason']='no predicted improvement';continue
            item['ok']=True;item['reason']='safe predicted rejoin'
            item['score']=3.*forward+min(room,2.5)-1.4*dev-1.2*herr-.20*d
    viable=[c for c in candidates if c['ok']]
    return (max(viable,key=lambda c:c['score']) if viable else None),candidates
