import numpy as np

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
    # http://www.kostasalexis.com/frame-rotations-and-representations.html
    M, N = quat.shape
    ret = np.zeros((M, 3))
    ret[:, 0] = np.arctan2( (2*(quat[:,0]*quat[:,1] + quat[:,2]*quat[:,3])),
                            (np.square(quat[:,3]) + np.square(quat[:,0]) - np.square(quat[:,1]) - np.square(quat[:,2])))

    ret[:, 1] = np.arcsin(2*((quat[:,0] * quat[:,2]) - (quat[:,1] * quat[:,3])))

    ret[:, 2] = np.arctan2( (2*(quat[:,0]*quat[:,3]+quat[:,1]*quat[:,2])),
                            (np.square(quat[:,0]) + np.square(quat[:,1]) - np.square(quat[:,2]) - np.square(quat[:,3])))

    return ret

def quatconj(q):
    w = q[:,0]
    x = q[:,1]
    y = q[:,2]
    z = q[:,3]
    return np.stack([w, -x, -y, -z], axis=-1)

def quatrotate_matlab(q,v):
    q0 = q[:,0]
    q1 = q[:,1]
    q2 = q[:,2]
    q3 = q[:,3]

    mat = np.array([[1-2*np.square(q2)-2*np.square(q3), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2)],
                    [2*(q1*q2-q0*q3), 1-2*np.square(q1)-2*np.square(q3), 2*(q2*q3+q0*q1)],
                    [2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), 1-2*np.square(q1)-2*np.square(q2)]])
    v_rot = np.empty(v.shape)
    for i in range(v.shape[1]):
        v_rot[:,i] = np.matmul(mat[:,:,i], v[:,i])
    return np.transpose(v_rot)

def quatexp(x):
    a, b, c, d = x
    cons = np.exp(a)
    v = np.array([b, c, d])
    nv = np.linalg.norm(v)
    ret = np.zeros(4)
    ret[0] = np.cos(nv)
    ret[1:] = v / nv * np.sin(nv)
    return np.nan_to_num(np.array([ret * cons]))

def quatlog(q):
    # https://math.stackexchange.com/questions/2552/the-logarithm-of-quaternion/2554#2554
    theta = np.arccos(q[:,0])
    v = q[:,1:4] / np.expand_dims(np.sin(theta),axis=-1)
    an = np.empty(v.shape)
    for i in range(v.shape[0]):
        an[i,:] = np.dot(v[i,:], theta[i])
    M = an.shape[0]
    ret = np.zeros((M, 4))
    ret[:,1:] = an
    return ret

#################################################################################
# def quat2eulxyz(quat):
#     psi = np.arctan2(2*(quat[:,0]*quat[:,3]-quat[:,1]*quat[:,2]),
#             np.square(quat[:,3])-np.square(quat[:,0]) - \
#                     np.square(quat[:,1])+np.square(quat[:,2]))
#     theta=np.arcsin(2*(quat[:,0]*quat[:,2]+quat[:,1]*quat[:,3]))
#     phi=np.arctan2(2*(quat[:,2]*quat[:,3]-quat[:,0]*quat[:,1]),
#             np.square(quat[:,3])+np.square(quat[:,0]) - \
#                     np.square(quat[:,1])-np.square(quat[:,2]))
#     return np.stack([psi,theta,phi], axis=-1)
#
# def eul2quatxyz(eul):
#     c1 = np.cos(eul[:,0]/2)
#     c2 = np.cos(eul[:,1]/2)
#     c3 = np.cos(eul[:,2]/2)
#     s1 = np.sin(eul[:,0]/2)
#     s2 = np.sin(eul[:,1]/2)
#     s3 = np.sin(eul[:,2]/2)
#     c13 = np.cos((eul[:,0]+ eul[:,2])/2)
#     s13 = np.sin((eul[:,0]+ eul[:,2])/2)
#     c1_3 = np.cos((eul[:,0]-eul[:,2])/2)
#     s1_3 = np.sin((eul[:,0]-eul[:,2])/2)
#     c3_1 = np.cos((eul[:,2]-eul[:,0])/2)
#     s3_1 = np.sin((eul[:,2]-eul[:,0])/2)
#     quat = np.stack([s1*c2*c3 + c1*s2*s3,
#                      c1*s2*c3-s1*c2*s3,
#                      c1*c2*s3 + s1*s2*c3,
#                      c1*c2*c3 - s1*s2*s3],axis=-1)
#
#     return quat/np.expand_dims(np.sqrt(np.sum(np.square(quat),axis=-1)), axis=-1)





# if __name__ == '__main__':
#     print(quatexp([0,0,0.7854,0]))
