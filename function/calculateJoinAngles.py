#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

"""
from quaterion_quality import quatmultiply, quatconj, quat2eulzyx
import numpy as np
import math

class JoinAngles(object):
    def __init__(self, segcos):
        def create_tmp(x):
            return quat2eulzyx(quatmultiply(quatconj(segcos.pelvis), x))
        tmp = create_tmp(segcos.rthigh)
        self.rhip = np.stack([tmp[:, 0], tmp[:, 2], tmp[:, 1]], axis=-1) * 180 / math.pi
        tmp = create_tmp(segcos.lthigh)
        self.lhip = np.stack([tmp[:, 0], -tmp[:, 2], -tmp[:, 1]], axis=-1) * 180 / math.pi
        tmp = create_tmp(segcos.rshank)
        self.rknee = np.stack([-tmp[:, 0], tmp[:, 2], tmp[:, 1]], axis=-1) * 180 / math.pi
        tmp = create_tmp(segcos.lshank)
        self.lknee = np.stack([-tmp[:,0], -tmp[:,2], -tmp[:,1]], axis=-1) * 180/math.pi
        tmp = create_tmp(segcos.rfoot)
        self.rankle = np.stack([tmp[:,0],tmp[:,2],tmp[:,1]], axis=-1) * 180 / math.pi
        tmp = create_tmp(segcos.lfoot)
        self.lankle = np.stack([tmp[:,0],-tmp[:,2],-tmp[:,1]], axis=-1) * 180 / math.pi

def make_jointangles(segcos):
    return JoinAngles(segcos)

if __name__ == '__main__':
    pass
