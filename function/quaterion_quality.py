#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import numpy as np
from helpers import *
# from squaternion import euler2quat, quat2euler, Quaternion

def quatmultiply(quaternion1, quaternion0):
    # w0, x0, y0, z0 = quaternion0
    w0 = quaternion0[:,0]
    x0 = quaternion0[:,1]
    y0 = quaternion0[:,2]
    z0 = quaternion0[:,3]

    w1 = quaternion1[:,0]
    x1 = quaternion1[:,1]
    y1 = quaternion1[:,2]
    z1 = quaternion1[:,3]

    # w1, x1, y1, z1 = quaternion1
    return np.stack([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], axis=-1)

def quat2eulzyx(quat):
    M, N = quat.shape
    ret = np.zeros((M, 3))
    ret[:, 0] = np.arctan2(2*(quat[:,0]*quat[:,1]+quat[:,2]*quat[:,3]),
            np.square(quat[:,3])+np.square(quat[:,0]) - \
                    np.square(quat[:,1])-np.square(quat[:,2]))
    ret[:, 1] = np.arcsin(2*(quat[:,1] * quat[:,3] - quat[:,0] * quat[:,2]))
    ret[:, 2] = np.arctan2(2*(quat[:,0]*quat[:,3]+quat[:,2]*quat[:,1]),
            np.square(quat[:,3]) - np.square(quat[:,0])- \
                    np.square(quat[:,1]) + np.square(quat[:,2]))
    return ret

def quat2eulxyz(quat):
    psi = np.arctan2(2*(quat[:,0]*quat[:,3]-quat[:,1]*quat[:,2]),
            np.square(quat[:,3])-np.square(quat[:,0]) - \
                    np.square(quat[:,1])+np.square(quat[:,2]))
    theta=np.arcsin(2*(quat[:,0]*quat[:,2]+quat[:,1]*quat[:,3]))
    phi=np.arctan2(2*(quat[:,2]*quat[:,3]-quat[:,0]*quat[:,1]),
            np.square(quat[:,3])+np.square(quat[:,0]) - \
                    np.square(quat[:,1])-np.square(quat[:,2]))
    return np.stack([psi,theta,phi], axis=-1)

def eul2quatxyz(eul):
    c1 = np.cos(eul[:,0]/2)
    c2 = np.cos(eul[:,1]/2)
    c3 = np.cos(eul[:,2]/2)
    s1 = np.sin(eul[:,0]/2)
    s2 = np.sin(eul[:,1]/2)
    s3 = np.sin(eul[:,2]/2)
    c13 = np.cos((eul[:,0]+ eul[:,2])/2)
    s13 = np.sin((eul[:,0]+ eul[:,2])/2)
    c1_3 = np.cos((eul[:,0]-eul[:,2])/2)
    s1_3 = np.sin((eul[:,0]-eul[:,2])/2)
    c3_1 = np.cos((eul[:,2]-eul[:,0])/2)
    s3_1 = np.sin((eul[:,2]-eul[:,0])/2)
    quat = np.stack([s1*c2*c3 + c1*s2*s3,
                     c1*s2*c3-s1*c2*s3,
                     c1*c2*s3 + s1*s2*c3,
                     c1*c2*c3 - s1*s2*s3],axis=-1)

    return quat/np.expand_dims(np.sqrt(np.sum(np.square(quat),axis=-1)), axis=-1)

def quatconj(q):
    w = q[:,0]
    x = q[:,1]
    y = q[:,2]
    z = q[:,3]
    return np.stack([w, -x, -y, -z], axis=-1)


def quatRotate(q, trans):
    xx = q[:,1] * q[:,1]
    xy = q[:,1] * q[:,2]
    xz = q[:,1] * q[:,3]
    xw = q[:,1] * q[:,0]
    yy = q[:,2] * q[:,2]
    yz = q[:,2] * q[:,3]
    yw = q[:,2] * q[:,0]
    zz = q[:,3] * q[:,3]
    zw = q[:,3] * q[:,0]
    return 2*np.stack([(0.5-yy-zz)*trans[0]+(xy-zw)*trans[1]+(xz+yw)*trans[2],
        (xy+zw)*trans[0]+(0.5-xx-zz)*trans[1]+(yz-xw)*trans[2],
        (xz-yw)*trans[0]+(yz+xw)*trans[1]+(0.5-xx-yy)*trans[2]], axis=-1)

def quatRotate2(q, trans):
    xx = q[:,1] * q[:,1]
    xy = q[:,1] * q[:,2]
    xz = q[:,1] * q[:,3]
    xw = q[:,1] * q[:,0]
    yy = q[:,2] * q[:,2]
    yz = q[:,2] * q[:,3]
    yw = q[:,2] * q[:,0]
    zz = q[:,3] * q[:,3]
    zw = q[:,3] * q[:,0]
    return 2*np.stack([(0.5-yy-zz)*trans[:,0]+(xy-zw)*trans[:,1]+(xz+yw)*trans[:,2],
        (xy+zw)*trans[:,0]+(0.5-xx-zz)*trans[:,1]+(yz-xw)*trans[:,2],
        (xz-yw)*trans[:,0]+(yz+xw)*trans[:,1]+(0.5-xx-yy)*trans[:,2]], axis=-1)


def quatlog(q):
    theta = np.arccos(q[:,0])
    v = q[:,1:4] / np.expand_dims(np.sin(theta),axis=-1)
    an = np.expand_dims(theta, axis=-1) * v
    M = an.shape[0]
    ret = np.zeros((M, 4))
    ret[:,1:] = an
    return ret

def quatexp(x):
    a, b, c, d = x
    cons = np.exp(a)
    v = np.array([b, c, d])
    nv = np.linalg.norm(v)
    ret = np.zeros(4)
    ret[0] = np.cos(nv)
    ret[1:] = v / nv * np.sin(nv)
    return np.array([ret * cons])

if __name__ == '__main__':
    print(quatexp([0,0,0.7854,0]))
