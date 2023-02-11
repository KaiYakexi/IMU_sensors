#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import viscid
from helpers import *
from calculateJoinOrigins import *

class Segcos(object):
    def __init__(self, markers, jointorigins):
        # Calculate pelvis coordinate system
        PSI = markers['RPSI'] + 0.5 * (markers['LPSI'] - markers['RPSI'])
        M, N = markers['RASI'].shape
        tmpCOS = np.zeros([M, N, 3])
        tmp = np.cross((markers['RASI']-PSI),(markers['LASI']-PSI), axis=1)
        tmpCOS[:,:,2] = markers['RASI'] - markers['LASI']
        tmpCOS[:,:,0] = np.cross(tmp, tmpCOS[:,:,2], axis=1)
        tmpCOS[:,:,1] = np.cross(tmpCOS[:,:,2], tmpCOS[:,:,1], axis=1)
        tmpCOS = tmpCOS / np.expand_dims(np.sqrt(np.sum(tmpCOS**2, axis=-1)), axis=-1)
        self.pelvis = np.transpose(np.transpose(tmpCOS,[2,1,0]), [1, 0, 2])

        # Calculate thigh coordinate systems
        tmpCOS[:,:,1] = jointorigins.rhip - jointorigins.rknee
        tmp = np.cross(jointorigins.rhip - markers['RKNE'], jointorigins.rhip-markers['RKNEM'], axis=1)
        tmpCOS[:,:,2] = np.cross(tmp, tmpCOS[:,:,1], axis=1)
        tmpCOS[:,:,0] = np.cross(tmpCOS[:,:,1], tmpCOS[:,:,2], axis=1)
        tmpCOS = tmpCOS / np.expand_dims(np.sqrt(np.sum(tmpCOS**2, axis=-1)), axis=-1)
        self.rthigh = np.transpose(np.transpose(tmpCOS,[2,1,0]), [1, 0, 2])

        # Left thigh coordinate system (Proximal = Wu)
        tmpCOS[:,:,1] = jointorigins.lhip-jointorigins.lknee
        tmp = np.cross( (jointorigins.lhip-markers['LKNEM']), (jointorigins.lhip-markers['LKNE']), axis=1)
        tmpCOS[:,:,2] = np.cross(tmp, tmpCOS[:,:,1], axis=1)
        tmpCOS[:,:,0] = np.cross(tmpCOS[:,:,1] , tmpCOS[:,:,2], axis=1)
        tmpCOS = tmpCOS / np.expand_dims(np.sqrt(np.sum(tmpCOS**2, axis=-1)), axis=-1)
        self.lthigh = np.transpose(np.transpose(tmpCOS,[2,1,0]) , [1,0,2])

        # Calculate shank coordinate sytems
        tmpCOS[:,:,2] = markers['RANKM']-markers['RANK']
        tmpCOS[:,:,0] = np.cross( (markers['RANK']-jointorigins.rknee), (markers['RANKM']-jointorigins.rknee), axis=1)
        tmpCOS[:,:,1] = np.cross(tmpCOS[:,:,2] , tmpCOS[:,:,0], axis=1)
        tmpCOS = tmpCOS / np.expand_dims(np.sqrt(np.sum(tmpCOS**2, axis=-1)), axis=-1)
        self.rshank = np.transpose(np.transpose(tmpCOS,[2,1,0]) , [1,0,2])

        tmpCOS[:,:,2] = markers['LANKM']-markers['LANK']
        tmpCOS[:,:,0] = np.cross( (markers['LANKM']-jointorigins.lknee), (markers['LANK']-jointorigins.lknee), axis=1)
        tmpCOS[:,:,1] = np.cross(tmpCOS[:,:,2] , tmpCOS[:,:,0], axis=1)
        tmpCOS = tmpCOS / np.expand_dims(np.sqrt(np.sum(tmpCOS**2, axis=-1)), axis=-1)
        self.lshank = np.transpose(np.transpose(tmpCOS,[2,1,0]) , [1,0,2])

        # Calculate foot coordinate sytems
        tmpCOS[:,:,0] = markers['RTOE']-markers['RHEEL']
        tmpCOS[:,:,1] = np.cross((markers['RCAL']-markers['RCALM']), tmpCOS[:,:,0], axis=1)
        tmpCOS[:,:,2] = np.cross(tmpCOS[:,:,0] , tmpCOS[:,:,1] ,axis=1)
        tmpCOS = tmpCOS / np.expand_dims(np.sqrt(np.sum(tmpCOS**2, axis=-1)), axis=-1)
        self.rfoot = np.transpose(np.transpose(tmpCOS,[2,1,0]) , [1,0,2])
        tmpCOS[:,:,0] = markers['LTOE']-markers['LHEEL']
        tmpCOS[:,:,1] = np.cross( tmpCOS[:,:,0] , (markers['LCAL']-markers['LCALM']),axis=1)
        tmpCOS[:,:,2] = np.cross( tmpCOS[:,:,0] , tmpCOS[:,:,1],axis=1)
        tmpCOS = tmpCOS / np.expand_dims(np.sqrt(np.sum(tmpCOS**2, axis=-1)), axis=-1)
        self.lfoot = np.transpose(np.transpose(tmpCOS,[2,1,0]) , [1,0,2])

        # Transform from 3x3xn matrices to nX4 quaternions
        def to_quat(x):
            x = np.einsum('ijk->kij', x)
            x = viscid.rotm2quat(x)
            return x

        self.pelvis = to_quat(self.pelvis)
        self.rthigh = to_quat(self.rthigh)
        self.lthigh = to_quat(self.lthigh)
        self.rshank = to_quat(self.rshank)
        self.lshank = to_quat(self.lshank)
        self.rfoot = to_quat(self.rfoot)
        self.lfoot = to_quat(self.lfoot)

def make_segcos(markers, joint_origins):
    return Segcos(markers, joint_origins)

if __name__ == '__main__':
    acq = get_acq('./Hi_Be_1_Gang_01.c3d')
    markers = get_markers(acq)
    join_origins = make_join_origins(markers)
    segcos = make_segcos(markers, join_origins)
    # print(segcos.pelvis.shape)
