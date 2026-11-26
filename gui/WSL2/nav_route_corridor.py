#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Planar *diagnostic* swept-route proximity geometry (no ROS imports).

This corridor is a supplementary VLP-16 region-of-interest only. It is NOT a
collision-avoidance system. The measured, instrumented Hunter footprint must be
validated before the result can be used for real motion. Never treat a point
outside this corridor as proof that the space is safe.
"""
from __future__ import division
import math


def project_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    u = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den)) if den > 1e-9 else 0.0
    cx, cy = ax + u * dx, ay + u * dy
    return math.hypot(px - cx, py - cy), u, (cx, cy)


def route_to_base(route, next_index, pose_x, pose_y, yaw, horizon_m, max_start_segments=5):
    """Follow the not-yet-completed path, starting at its closest local segment.

    Does not jump back to consumed waypoints. A small robot-to-route connector
    is deliberately included so obstacles between the chassis and route are
    not accidentally excluded. Returns a polyline in base_link coordinates.
    """
    if route is None or len(route) < 2:
        return None
    vals = (pose_x, pose_y, yaw, horizon_m)
    if not all(not (math.isnan(float(a)) or math.isinf(float(a))) for a in vals) or horizon_m <= 0:
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    first = max(0, min(len(route) - 2, int(next_index) - 1))
    coords = []
    for wp in route[first:]:
        dx, dy = float(wp['x']) - pose_x, float(wp['y']) - pose_y
        coords.append((c * dx + s * dy, -s * dx + c * dy))
    best = None
    # Restrict nearest-segment selection to upcoming segments, not a distant
    # crossing of a later part of the route.
    for i in range(min(max_start_segments, len(coords) - 1)):
        a, b = coords[i], coords[i + 1]
        dist, t, proj = project_segment(0.0, 0.0, a[0], a[1], b[0], b[1])
        if best is None or dist < best[0]:
            best = (dist, i, proj)
    if best is None:
        return None
    # If the controller is >0.5m off the path, healthy() rejects movement.
    # For diagnostics the original chassis location remains in the swept area.
    path = [(0.0, 0.0)]
    if math.hypot(*best[2]) >= 0.03:
        path.append(best[2])
    path.extend(coords[best[1] + 1:])
    # Do not extend the path past the goal; stop geometry at the last waypoint.
    clipped = [path[0]]
    remaining = float(horizon_m)
    for pt in path[1:]:
        last = clipped[-1]
        seg = math.hypot(pt[0] - last[0], pt[1] - last[1])
        if seg < 1e-5:
            continue
        if seg >= remaining:
            t = remaining / seg
            clipped.append((last[0] + (pt[0] - last[0]) * t,
                            last[1] + (pt[1] - last[1]) * t))
            break
        clipped.append(pt)
        remaining -= seg
    return clipped if len(clipped) >= 2 else None


def path_nearest(point_x, point_y, path):
    """Return (absolute lateral distance, route arc length, signed lateral offset)."""
    if path is None or len(path) < 2:
        return None
    best = None
    along = 0.0
    for a, b in zip(path[:-1], path[1:]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-6:
            continue
        d, t, _ = project_segment(point_x, point_y, a[0], a[1], b[0], b[1])
        if best is None or d < best[0]:
            signed = ((b[0] - a[0]) * (point_y - a[1]) -
                      (b[1] - a[1]) * (point_x - a[0])) / seg
            best = (d, along + t * seg, signed)
        along += seg
    return best


def in_corridor(x, y, path, half_width_m, min_forward_m=0.5, max_forward_m=None):
    """True only when a point is in the actual upcoming swept route strip."""
    if half_width_m <= 0 or x <= min_forward_m:
        return (False, None)
    if max_forward_m is not None and x >= max_forward_m:
        return (False, None)
    nearest = path_nearest(x, y, path)
    if nearest is None:
        return (False, None)
    return (nearest[0] <= half_width_m, nearest)


def corridor_edges(path, half_width_m):
    """Visualization-only approximate strip edges (segment tangent normals)."""
    if path is None or len(path) < 2:
        return [], []
    left, right = [], []
    for i, pt in enumerate(path):
        prev = path[max(0, i - 1)]
        after = path[min(len(path) - 1, i + 1)]
        dx, dy = after[0] - prev[0], after[1] - prev[1]
        n = math.hypot(dx, dy)
        if n < 1e-6:
            continue
        nx, ny = -dy / n, dx / n
        left.append((pt[0] + nx * half_width_m, pt[1] + ny * half_width_m))
        right.append((pt[0] - nx * half_width_m, pt[1] - ny * half_width_m))
    return left, right


def classify_point_region(x, y, path, half_width_m, immediate_front_m, horizon_m):
    """Classify a scan return without treating every forward-side return as a stop.

    Returns (region, projection): 'body', 'route', 'side', or 'invalid'.
    'side' means *outside the provisional route ROI*, NOT verified collision-free.
    The near-body forward guard is always checked independently of the route.
    No map-static object, including a wall, is automatically whitelisted.
    """
    if path is None or len(path) < 2:
        return 'invalid', None
    if math.isnan(float(x)) or math.isinf(float(x)):
        return 'invalid', None
    if x <= 0.5 or x >= horizon_m:
        return 'invalid', None
    on_route, projection = in_corridor(
        x, y, path, half_width_m, max_forward_m=horizon_m)
    # The current robot is not guaranteed to be perfectly on the route.
    # A short body-centered region must still stop for imminent collision.
    if x <= immediate_front_m and abs(y) < half_width_m:
        return 'body', projection
    if on_route:
        return 'route', projection
    return 'side', projection
