#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Command prediction for the user's uploaded hunter_ws (ROS1).
Twist.angular.z is signed yaw rate; AngelVelocity2Angel performs the
Ackermann conversion in hunter_messenger.cpp. Do NOT send steer angle.
"""
from __future__ import division
import math

# Actual uploaded hunter_params.hpp; protocol version is selected in C++.
MODELS={'hunter2':(.650,.605,.461,.580),
        'hunter1':(.650,.578,.374,.444),
        'hunter':(.650,.578,.374,.444)}

class HunterSteering(object):
    def __init__(self, model):
        self.model=str(model).lower()
        if self.model not in MODELS:
            raise ValueError('Unknown Hunter model: '+self.model)
        self.wheelbase,self.track,self.max_angle,self.max_inner=MODELS[self.model]
        self.min_radius=max(self.wheelbase/math.tan(self.max_angle),
                            self.wheelbase/math.tan(self.max_inner)+self.track/2.,
                            self.wheelbase+0.01)
        self.max_curvature=1./self.min_radius

    def apply(self, curvature, velocity):
        """Return predicted central angle, applied curvature, signed yaw rate.
        Bound requested curvature before the driver's high-curvature branch,
        which omits the reverse sign correction. Both directions use yaw=v*k.
        """
        if not all(not(math.isnan(float(v)) or math.isinf(float(v))) for v in (curvature,velocity)):
            raise ValueError('Non-finite Hunter command')
        applied=max(-self.max_curvature,min(self.max_curvature,curvature))
        angle=math.atan(self.wheelbase*applied)
        return angle,applied,velocity*applied
