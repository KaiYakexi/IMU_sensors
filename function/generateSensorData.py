#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8

"""

"""
from generateAccelerometerData import generateAccelerometerData
from generateGyroscopeData import generateGyroscopeData

class Accelerometer(object):
    def __init__(self, sensorposition,sensororientation,frequency):
        self.pelvis = generateAccelerometerData(sensorposition.pelvis,sensororientation.pelvis,frequency )
        self.rthigh = generateAccelerometerData( sensorposition.rthigh,sensororientation.rthigh,frequency )
        self.lthigh = generateAccelerometerData( sensorposition.lthigh,sensororientation.lthigh,frequency )
        self.rshank = generateAccelerometerData( sensorposition.rshank,sensororientation.rshank,frequency )
        self.lshank = generateAccelerometerData( sensorposition.lshank,sensororientation.lshank,frequency )
        self.rfoot = generateAccelerometerData( sensorposition.rfoot,sensororientation.rfoot,frequency )
        self.lfoot = generateAccelerometerData( sensorposition.lfoot,sensororientation.lfoot,frequency )


class Gyroscope(object):
    def __init__(self, sensorposition,sensororientation,frequency):
        self.pelvis = generateGyroscopeData( sensororientation.pelvis,frequency )
        self.rthigh = generateGyroscopeData( sensororientation.rthigh,frequency )
        self.lthigh = generateGyroscopeData( sensororientation.lthigh,frequency )
        self.rshank = generateGyroscopeData( sensororientation.rshank,frequency )
        self.lshank = generateGyroscopeData( sensororientation.lshank,frequency )
        self.rfoot = generateGyroscopeData( sensororientation.rfoot,frequency )
        self.lfoot = generateGyroscopeData( sensororientation.lfoot,frequency )

def make_sensordata(sensorposition, sensororientation, frequency):
    return Accelerometer(sensorposition, sensororientation, frequency), Gyroscope(sensorposition, sensororientation, frequency)
