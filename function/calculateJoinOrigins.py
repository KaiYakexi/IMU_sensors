#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
import numpy as np
from helpers import *

class JoinOrigins(object):
    def __init__(self, markers):
        self.rankle = markers['RANK'] + 0.5 * (markers['RANKM'] - markers['RANK'])
        self.lankle = markers['LANK'] + 0.5 * (markers['LANKM'] - markers['LANK'])
        self.rknee = markers['RKNE'] + 0.5 * (markers['RKNEM'] - markers['RKNE'])
        self.lknee = markers['LKNE'] + 0.5 * (markers['LKNEM'] - markers['LKNE'])
        PSI = markers['RPSI'] + 0.5 * (markers['LPSI'] - markers['RPSI'])
        ASI = markers['RASI'] + 0.5 * (markers['LASI'] - markers['RASI'])
        PelvisDepth = np.sqrt(np.sum((ASI - PSI) ** 2, axis=-1))
        PelvisWidth = np.sqrt(np.sum((markers['RASI'] - markers['LASI']) ** 2, axis=-1))
        LegLengthRight = np.sqrt(np.sum((markers['RASI'] - markers['RKNEM'])**2, axis=-1)) \
                + np.sqrt(np.sum((markers['RKNEM'] - markers['RANKM'])**2, axis=-1))
        LegLengthLeft = np.sqrt(np.sum((markers['LASI'] - markers['LKNEM'])**2, axis=-1)) \
                + np.sqrt(np.sum((markers['LKNEM'] - markers['LANKM'])**2, axis=-1))

        M, N = markers['RASI'].shape
        PelvisCOS = np.zeros((M, N, 3))
        PelvisCOS[:,:,2] = markers['RASI'] - markers['LASI']
        PelvisCOS[:,:,1] = np.cross(markers['RASI']-PSI, markers['LASI']-PSI, axis=1)
        PelvisCOS[:,:,0] = np.cross(PelvisCOS[:,:,1], PelvisCOS[:,:,2], axis=1)

        self.rhip = ASI+PelvisCOS[:,:,0] * np.array([(-0.24*PelvisDepth-9.9)]).T \
                    +PelvisCOS[:,:,1] * np.array([(-0.16*PelvisWidth-0.04*LegLengthRight-7.1)]).T \
                    +PelvisCOS[:,:,2] * np.array([(0.28*PelvisDepth+0.16*PelvisWidth+7.9)]).T

        self.lhip = ASI+PelvisCOS[:,:,0] * np.array([(-0.24*PelvisDepth-9.9)]).T \
                    +PelvisCOS[:,:,1] * np.array([(-0.16*PelvisWidth-0.04*LegLengthLeft-7.1)]).T \
                    -PelvisCOS[:,:,2] * np.array([(0.28*PelvisDepth+0.16*PelvisWidth+7.9)]).T


def make_join_origins(markers):
    return JoinOrigins(markers)

if __name__ == '__main__':
    acq = get_acq('./Hi_Be_1_Gang_01.c3d')
    markers = get_markers(acq)
    join_origins = make_join_origins(markers)
    # print(join_origins.rhip)
    print(join_origins.rankle)
    # print(join_origins.rhip)
    # print(join_origins.rhip)
    # print(join_origins.rhip)
