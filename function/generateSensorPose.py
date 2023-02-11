#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

from generateAccelerometerData import generateAccelerometerData
from generateGyroscopeData import generateGyroscopeData
from quaterion_quality import quatmultiply, quatconj, quat2eulxyz, quat2eulzyx, quatRotate, quatexp, eul2quatxyz
import numpy as np
from math import *
from helpers import *

class DefaultPosition(object):
    def __init__(self, markers,jointorigins,segcos,noise):
        markers = Markers(markers)
        self.pelvis = (markers.RPSI + markers.LPSI)/2 + quatRotate( quatconj(segcos.pelvis),[-20,0,0])
        self.rthigh = (jointorigins.rhip + jointorigins.rknee)/2 + quatRotate( quatconj(segcos.rthigh),[0,0,40])
        self.lthigh = (jointorigins.lhip + jointorigins.lknee)/2 + quatRotate( quatconj(segcos.lthigh),[0,0,-40])
        self.rshank = (jointorigins.rknee + jointorigins.rankle)/2 + quatRotate( quatconj(segcos.rshank),[20,60,-40])
        self.lshank = (jointorigins.lknee + jointorigins.lankle)/2 + quatRotate( quatconj(segcos.lshank),[20,60,45])
        self.rfoot = (markers.RTOE + markers.RHEEL)/2 + quatRotate(quatconj(segcos.rfoot),[20,20,0])
        self.lfoot = (markers.LTOE + markers.LHEEL)/2 + quatRotate(quatconj(segcos.lfoot),[20,20,0])


class DefaultOrientation(object):
    def __init__(self, markers,jointorigins,segcos,noise):
        self.pelvis = quatmultiply( quatmultiply( segcos.pelvis,quatexp([0,0,0,pi/4]) ),quatexp([0,pi/4,0,0]) )
        self.rthigh = quatmultiply( quatmultiply( segcos.rthigh,quatexp([0,pi/2,0,0]) ),quatexp([0,pi*1/24,0,0])  )
        self.lthigh = quatmultiply( quatmultiply( segcos.lthigh,quatexp([0,0,0,pi/2]) ),quatexp([0,pi*1/28,0,0]) )
        self.rshank = quatmultiply( quatmultiply( segcos.rshank,quatexp([0,pi*6/13,0,0]) ),quatexp([0,0,-pi*9/22,0]) )
        self.lshank = quatmultiply( quatmultiply( segcos.lshank,quatexp([0,pi*6/11,0,0]) ),quatexp([0,0,-pi*1/9,0]) )
        self.rfoot = quatmultiply( segcos.rfoot,quatexp([0,0,0,0]) )
        self.lfoot = quatmultiply( segcos.lfoot,quatexp([0,0,0,0]) )

class RelativePose(object):
    def __init__(self, noise):
        noiseMat = noise*np.random.rand(7,6)
        noiseMat[:,3:6] = noiseMat[:,3:6]*pi/180
        self.pelvis = noiseMat[0,:]
        self.rthigh = noiseMat[1,:]
        self.lthigh = noiseMat[2,:]
        self.rshank = noiseMat[3,:]
        self.lshank = noiseMat[4,:]
        self.rfoot = noiseMat[5,:]
        self.lfoot = noiseMat[6,:]

class SensorOrientation(object):
    def __init__(self, segcos,defaultorientation, relativepose):
        self.pelvis = segcos.pelvis
        self.rthigh = segcos.rthigh
        self.lthigh = segcos.lthigh
        self.rshank = segcos.rshank
        self.lshank = segcos.lshank
        self.rfoot = segcos.rfoot
        self.lfoot = segcos.lfoot
        self.pelvis = quatmultiply( defaultorientation.pelvis,eul2quatxyz(np.array([relativepose.pelvis[3:6]])))
        self.rthigh = quatmultiply( defaultorientation.rthigh,eul2quatxyz(np.array([relativepose.rthigh[3:6]])))
        self.lthigh = quatmultiply( defaultorientation.lthigh,eul2quatxyz(np.array([relativepose.lthigh[3:6]])))
        self.rshank = quatmultiply( defaultorientation.rshank,eul2quatxyz(np.array([relativepose.rshank[3:6]])))
        self.lshank = quatmultiply( defaultorientation.lshank,eul2quatxyz(np.array([relativepose.lshank[3:6]])))
        self.rfoot = quatmultiply( defaultorientation.rfoot,eul2quatxyz(np.array([relativepose.rfoot[3:6]])))
        self.lfoot = quatmultiply( defaultorientation.lfoot,eul2quatxyz(np.array([relativepose.lfoot[3:6]])))

class SensorPosition(object):
    def __init__(self, defaultposition,relativepose, segcos):
        self.pelvis = defaultposition.pelvis + quatRotate( quatconj(segcos.pelvis),relativepose.pelvis[0:3] )
        self.rthigh = defaultposition.rthigh + quatRotate( quatconj(segcos.rthigh),relativepose.rthigh[0:3] )
        self.lthigh = defaultposition.lthigh + quatRotate( quatconj(segcos.lthigh),relativepose.lthigh[0:3] )
        self.rshank = defaultposition.rshank + quatRotate( quatconj(segcos.rshank),relativepose.rshank[0:3] )
        self.lshank = defaultposition.lshank + quatRotate( quatconj(segcos.lshank),relativepose.lshank[0:3] )
        self.rfoot = defaultposition.rfoot + quatRotate( quatconj(segcos.rfoot),relativepose.rfoot[0:3] )
        self.lfoot = defaultposition.lfoot + quatRotate( quatconj(segcos.lfoot),relativepose.lfoot[0:3] )

def make_sensorpose(markers, jointorigins, segcos, noise):
    defaultposition = DefaultPosition(markers, jointorigins, segcos, noise)
    defaultorientation = DefaultOrientation(markers, jointorigins, segcos, noise)
    relativepose = RelativePose(noise)
    sensororientaton = SensorOrientation(segcos, defaultorientation, relativepose)
    sensorposition = SensorPosition(defaultposition, relativepose, segcos)
    return sensorposition, sensororientaton, relativepose
