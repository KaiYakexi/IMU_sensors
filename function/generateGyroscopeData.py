#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

"""

"""
import numpy as np
from scipy import signal as sg
from quaterion_quality import quatmultiply, quatconj, quatlog

def generateGyroscopeData(sensororientation,frequency):
    dt = 1/frequency;
    orientationOrder = 2;
    orientationFrames = 3;
    angvelocityOrder = 2;
    angvelocityFrames = 11;
    gyr = np.zeros(sensororientation.shape)
    q = sg.savgol_filter(sensororientation, orientationFrames, orientationOrder)

    q /= np.expand_dims(np.sqrt(np.sum(q**2, axis=1)), axis=-1)
    deltaq = quatmultiply(quatconj(q[:-1,:]), q[1:,:])
    tmp = 2*quatlog(deltaq) * frequency
    gyr = np.zeros((tmp.shape[0]+1, 3))
    gyr[:-1,:] = tmp[:,1:4]
    gyr[-1,:] = tmp[-1,1:4]
    return gyr
