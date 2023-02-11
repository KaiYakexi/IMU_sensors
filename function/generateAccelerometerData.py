#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

"""

"""

import numpy as np
from scipy import signal as sg
from quaterion_quality import quatRotate2

def generateAccelerometerData(sensorposition, sensororientation, frequency):
    dt = 1. / frequency

    positionOrder = 2
    positionFrames = 5
    velocityOrder = 2
    velocityFrames = 13
    accelerationOrder = 2
    accelerationFrames = 21
    acc = np.zeros(sensorposition.shape)
    pos = sensorposition
    pos = sg.savgol_filter(pos, positionFrames, positionOrder, axis=0) / 1000
    posGrad = np.gradient(pos, axis=1)
    vel = posGrad * frequency

    velGrad = np.gradient(vel, axis=1)
    acc = -velGrad * frequency

    acc[:,2] -= 9.81
    return quatRotate2(sensororientation, acc)
