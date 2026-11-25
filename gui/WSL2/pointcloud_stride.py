#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Preserve PointCloud2 fields, timestamp and point records while selecting
uniformly spaced records. Used only on the NDT Matching input stream.

The original /velodyne_points is NOT modified. Stride sampling is a performance
experiment, not a voxel filter; verify NDT match quality after changing stride.
"""
from __future__ import division
import numpy as np


def thinning_data(data, height, width, row_step, point_step, stride):
    stride = int(stride)
    if stride < 1 or stride > 8:
        raise ValueError('stride out of range')
    if height < 1 or width < 1 or point_step < 12 or row_step < width * point_step:
        raise ValueError('invalid cloud dimensions')
    if len(data) < row_step * height:
        raise ValueError('cloud byte data truncated')
    if stride == 1:
        # Also normalize possible row padding to a packed output.
        pass
    records = np.ndarray((height,width,point_step), dtype=np.uint8,
                         buffer=data, strides=(row_step,point_step,1))
    sampled = records[:, ::stride, :].copy(order='C')
    outwidth = sampled.shape[1]
    return sampled.tobytes(), outwidth, outwidth * point_step


def thin_cloud(cloud, stride, cloud_type=None):
    stride = int(stride)
    if stride == 1:
        return cloud
    payload, width, row_step = thinning_data(
        cloud.data, cloud.height, cloud.width, cloud.row_step,
        cloud.point_step, stride)
    output = (cloud_type or type(cloud))()
    output.header = cloud.header
    output.height = cloud.height
    output.width = width
    output.fields = cloud.fields
    output.is_bigendian = cloud.is_bigendian
    output.point_step = cloud.point_step
    output.row_step = row_step
    output.data = payload
    output.is_dense = cloud.is_dense
    return output
