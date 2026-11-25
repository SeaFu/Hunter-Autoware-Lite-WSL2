#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""ROS-free, testable policy helpers for live Hunter navigation.

A usable pose means a *new* valid NDT pose. Frequency is measured from wall
arrival times; stale/no updates cannot be counted as recovery. This module
never publishes to the vehicle.
"""
from __future__ import division
from collections import deque
import math


class PoseRateWindow(object):
    def __init__(self, timeout_s=1.0, min_hz=2.0, recovery_s=2.0):
        self.timeout_s=float(timeout_s)
        self.min_hz=float(min_hz)
        self.recovery_s=float(recovery_s)
        self.samples=deque(maxlen=100)
        self.last=None
        self.recovery_required=True

    def feed(self, now):
        now=float(now)
        if self.last is not None and now-self.last >= self.timeout_s:
            self.samples.clear()
            self.recovery_required=True
        if self.last is not None and now<=self.last:
            return
        self.last=now
        self.samples.append(now)
        while self.samples and now-self.samples[0]>max(2.5,self.recovery_s+0.5):
            self.samples.popleft()

    def age(self, now):
        return float('inf') if self.last is None else max(0.0,float(now)-self.last)

    def hz(self, now):
        if self.age(now)>=self.timeout_s or len(self.samples)<3:
            return 0.0
        span=self.samples[-1]-self.samples[0]
        return float(len(self.samples)-1)/span if span>0 else 0.0

    def ready(self, now):
        if self.age(now)>=self.timeout_s or len(self.samples)<3:
            return False
        span=self.samples[-1]-self.samples[0]
        if span < self.recovery_s or self.hz(now) < self.min_hz:
            return False
        # Guard against bursty packets: this isn't sufficient by itself for
        # safety, but prevents a 2-sample burst being interpreted as 2 fps.
        if any(b-a >= self.timeout_s for a,b in zip(list(self.samples)[:-1],list(self.samples)[1:])):
            return False
        self.recovery_required=False
        return True


def adaptive_speed_cap(configured_mps, exe_time_ms, pose_hz, reference_ms=100.0):
    """Conservative speed ceiling: slower NDT *or* slower pose rate slows motion.

    10 Hz, 100ms NDT => selected cruise target may be used.
    5 Hz, 200ms NDT => <=50% of target.
    2 Hz, 500ms NDT => <=20% of target.
    Further distance, lateral acceleration and freshness checks apply downstream.
    """
    values=(configured_mps,exe_time_ms,pose_hz,reference_ms)
    if not all(not (math.isnan(float(v)) or math.isinf(float(v))) for v in values):
        return 0.0
    if configured_mps<=0 or exe_time_ms<=0 or pose_hz<2.0 or reference_ms<=0:
        return 0.0
    compute=min(1.0,reference_ms/max(exe_time_ms,reference_ms))
    frequency=min(1.0,pose_hz/10.0)
    return max(0.0,min(float(configured_mps),configured_mps*compute,
                       configured_mps*frequency))


def required_clearance(speed_mps, reaction_s=1.2, decel_mps2=0.65, margin_m=1.5):
    """Distance required for reaction plus braking (model, NOT a brake guarantee)."""
    v=max(0.0,float(speed_mps))
    return float(margin_m)+v*float(reaction_s)+v*v/(2.0*float(decel_mps2))


def forward_speed_cap(obstacle_m, max_speed=5.0, reaction_s=1.2,
                      decel_mps2=0.65, margin_m=1.5):
    """Model-limited forward speed for an observed obstacle distance.

    This is NOT safety-rated: LiDAR visibility, actual deceleration, terrain,
    timestamps and moving objects must be independently validated.
    """
    if obstacle_m is None:
        return max(0.0,float(max_speed))
    vals=(obstacle_m,max_speed,reaction_s,decel_mps2,margin_m)
    if not all(not(math.isnan(float(x)) or math.isinf(float(x))) for x in vals):
        return 0.0
    if decel_mps2<=0 or reaction_s<0 or margin_m<0:return 0.0
    d=max(0.0,float(obstacle_m)-float(margin_m))
    a=float(decel_mps2)
    # solve v^2/(2a)+v*reaction <= available distance
    root=-a*float(reaction_s)+math.sqrt((a*float(reaction_s))**2+2*a*d)
    return max(0.0,min(float(max_speed),root))


class ClearanceGate(object):
    """Debounce a simple forward-ROI clear indication; false blocks immediately."""
    def __init__(self, clear_dwell_s=1.2, min_clear_frames=5):
        self.clear_dwell_s=float(clear_dwell_s)
        self.min_clear_frames=int(min_clear_frames)
        self.clear_since=None
        self.clear_frames=0

    def update(self, clear, now):
        if not clear:
            self.clear_since=None
            self.clear_frames=0
            return False
        if self.clear_since is None:
            self.clear_since=float(now)
        self.clear_frames+=1
        return (self.clear_frames>=self.min_clear_frames and
                float(now)-self.clear_since>=self.clear_dwell_s)

    def elapsed(self, now):
        return max(0.0, float(now)-self.clear_since) if self.clear_since is not None else 0.0
