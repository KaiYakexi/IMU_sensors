import scipy.io as spio
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
from scipy import signal as sg
from quaternion_quality import quatmultiply, quatconj, quat2eulzyx, quatrotate_matlab, quatexp, quatlog

def generateAccelerometerData(sensorposition, sensororientation, frequency=100, order=2, frames=5):

    pos = sg.savgol_filter(sensorposition, frames, order, axis=0, mode='interp') / 1000 #FIXME filter not good
    posGrad = np.gradient(pos, axis=0)
    # Calculate velocities
    vel = posGrad * frequency
    # Calculate accelerations
    velGrad = np.gradient(vel, axis=0)
    acc = -velGrad * frequency

    acc[:,2] -= 9.81
    return quatrotate_matlab(sensororientation, np.transpose(acc))

def make_accelerometer(sensorposition,sensororientation):
    accelerometer = dict()
    accelerometer['pelvis'] = generateAccelerometerData(sensorposition['pelvis'],sensororientation['pelvis'])
    accelerometer['rthigh'] = generateAccelerometerData(sensorposition['rthigh'],sensororientation['rthigh'])
    accelerometer['lthigh'] = generateAccelerometerData(sensorposition['lthigh'],sensororientation['lthigh'])
    accelerometer['rshank'] = generateAccelerometerData(sensorposition['rshank'] ,sensororientation['rshank'])
    accelerometer['lshank'] = generateAccelerometerData(sensorposition['lshank'],sensororientation['lshank'])
    accelerometer['rfoot'] = generateAccelerometerData(sensorposition['rfoot'],sensororientation['rfoot'])
    accelerometer['lfoot'] = generateAccelerometerData(sensorposition['lfoot'],sensororientation['lfoot'])
    return accelerometer

def generateGyroscopeData(sensororientation, frequency=100, order=2, frames=5):

    q = sg.savgol_filter(sensororientation, frames, order,  axis=0, mode='interp')
    q /= np.expand_dims(np.sqrt(np.sum(q**2, axis=1)), axis=-1)
    temp = quatconj(q[:-1,:])
    deltaq = quatmultiply(temp, q[1:,:])
    tmp = 2*quatlog(deltaq) * frequency
    gyr = np.zeros((tmp.shape[0]+1, 3))
    gyr[:-1,:] = tmp[:,1:4]
    gyr[-1,:] = tmp[-1,1:4]
    return gyr

def make_gyroscope(sensororientation,frequency=100):
    gyroscope = dict()
    gyroscope['pelvis'] = generateGyroscopeData( sensororientation['pelvis'],frequency )
    gyroscope['rthigh'] = generateGyroscopeData( sensororientation['rthigh'],frequency )
    gyroscope['lthigh'] = generateGyroscopeData( sensororientation['lthigh'],frequency )
    gyroscope['rshank'] = generateGyroscopeData( sensororientation['rshank'],frequency )
    gyroscope['lshank'] = generateGyroscopeData( sensororientation['lshank'],frequency )
    gyroscope['rfoot'] = generateGyroscopeData( sensororientation['rfoot'],frequency )
    gyroscope['lfoot'] = generateGyroscopeData( sensororientation['lfoot'],frequency )
    return gyroscope

#######################
def make_position(m, jo, segcos):
    defaultposition = dict()
    defaultposition['pelvis'] = (m['RPSI'] + m['LPSI']) / 2 + quatrotate_matlab(quatconj(segcos.pelvis),
        np.repeat(np.expand_dims(np.array([-20, 0, 0]), axis=1), np.shape(segcos.pelvis)[0], axis=1))
    defaultposition['rthigh'] = (jo.rhip + jo.rknee) / 2 + quatrotate_matlab(quatconj(segcos.rthigh),
        np.repeat(np.expand_dims(np.array([0, 0, 40]), axis=1), np.shape(segcos.pelvis)[0], axis=1))
    defaultposition['lthigh'] = (jo.lhip + jo.lknee) / 2 + quatrotate_matlab(quatconj(segcos.lthigh),
        np.repeat(np.expand_dims(np.array([0, 0, -40]), axis=1), np.shape(segcos.pelvis)[0], axis=1))
    defaultposition['rshank'] = (jo.rknee + jo.rankle) / 2 + quatrotate_matlab(quatconj(segcos.rshank),
        np.repeat(np.expand_dims(np.array([20, 60, -40]), axis=1), np.shape(segcos.pelvis)[0], axis=1))
    defaultposition['lshank'] = (jo.lknee + jo.lankle) / 2 + quatrotate_matlab(quatconj(segcos.lshank),
        np.repeat(np.expand_dims(np.array([20, 60, 45]), axis=1), np.shape(segcos.pelvis)[0], axis=1))
    defaultposition['rfoot'] = (m['RTOE'] + m['RHEEL']) / 2 + quatrotate_matlab(quatconj(segcos.rfoot),
        np.repeat(np.expand_dims(np.array([20, 20, 0]), axis=1), np.shape(segcos.pelvis)[0], axis=1))
    defaultposition['lfoot'] = (m['LTOE'] + m['LHEEL']) / 2 + quatrotate_matlab(quatconj(segcos.lfoot),
        np.repeat(np.expand_dims(np.array([20, 20, 0]), axis=1), np.shape(segcos.pelvis)[0], axis=1))
    return defaultposition

def make_orientation(segcos):
    defaultorientation = dict()
    defaultorientation['pelvis']  = quatmultiply(
        quatmultiply( segcos.pelvis,quatexp([0,0,0,math.pi/4]) ),quatexp([0,math.pi/4,0,0]) )
    defaultorientation['rthigh'] = quatmultiply(
        quatmultiply( segcos.rthigh,quatexp([0,math.pi/2,0,0]) ),quatexp([0,math.pi*1/24,0,0])  )
    defaultorientation['lthigh'] = quatmultiply(
        quatmultiply( segcos.lthigh,quatexp([0,0,0,math.pi/2]) ),quatexp([0,math.pi*1/28,0,0]) )
    defaultorientation['rshank'] = quatmultiply(
        quatmultiply( segcos.rshank,quatexp([0,math.pi*6/13,0,0]) ),quatexp([0,0,-math.pi*9/22,0]) )
    defaultorientation['lshank'] = quatmultiply(
        quatmultiply( segcos.lshank,quatexp([0,math.pi*6/11,0,0]) ),quatexp([0,0,-math.pi*1/9,0]) )
    defaultorientation['rfoot'] = quatmultiply( segcos.rfoot,quatexp([0,0,0,0]) )
    defaultorientation['lfoot'] = quatmultiply( segcos.lfoot,quatexp([0,0,0,0]) )
    return defaultorientation

class JointAngles(object):
    def __init__(self, segcos):
        def create_tmp(x, y):
            return quat2eulzyx(quatmultiply(quatconj(x), y))

        # Hip joint angles
        #  Right
        tmp = create_tmp(segcos.pelvis, segcos.rthigh)
        self.rhip = np.stack([tmp[:, 2], tmp[:, 0], tmp[:, 1]], axis=-1) * 180 / math.pi
        #  Left
        tmp = create_tmp(segcos.pelvis, segcos.lthigh)
        self.lhip = np.stack([tmp[:, 2], -tmp[:, 0], -tmp[:, 1]], axis=-1) * 180 / math.pi
        # Knee joint angles
        #  Right
        tmp = create_tmp(segcos.rthigh, segcos.rshank)
        self.rknee = np.stack([-tmp[:, 2], tmp[:, 0], tmp[:, 1]], axis=-1) * 180 / math.pi
        #  Left
        tmp = create_tmp(segcos.lthigh, segcos.lshank)
        self.lknee = np.stack([-tmp[:,2], -tmp[:,0], -tmp[:,1]], axis=-1) * 180/math.pi
        # Ankle joint angles
        #  Right
        tmp = create_tmp(segcos.rshank, segcos.rfoot)
        self.rankle = np.stack([tmp[:,2],tmp[:,0],tmp[:,1]], axis=-1) * 180 / math.pi
        #  Left
        tmp = create_tmp(segcos.lshank, segcos.lfoot)
        self.lankle = np.stack([tmp[:,2],-tmp[:,0],-tmp[:,1]], axis=-1) * 180 / math.pi

def make_jointangles(segcos):
    return JointAngles(segcos)

class SegmentCoordinateSystem(object):
    def __init__(self, markers, jointorigins):
        # Calculate pelvis coordinate system
        PSI = markers['RPSI'] + 0.5 * (markers['LPSI'] - markers['RPSI'])
        M, N = markers['RASI'].shape
        tmpCOS = np.zeros([M, N, 3])
        tmp = np.cross((markers['RASI']-PSI),(markers['LASI']-PSI), axis=1)
        tmpCOS[:,:,2] = markers['RASI'] - markers['LASI']
        tmpCOS[:,:,0] = np.cross(tmp, tmpCOS[:,:,2], axis=1)
        tmpCOS[:,:,1] = np.cross(tmpCOS[:,:,2], tmpCOS[:,:,0], axis=1)
        self.pelvis = tmpCOS / np.expand_dims(np.linalg.norm(tmpCOS, axis=1), axis=1)

        # Calculate thigh coordinate systems
        tmpCOS[:,:,1] = jointorigins.rhip - jointorigins.rknee
        tmp = np.cross(jointorigins.rhip - markers['RKNE'], jointorigins.rhip-markers['RKNEM'], axis=1)
        tmpCOS[:,:,2] = np.cross(tmp, tmpCOS[:,:,1], axis=1)
        tmpCOS[:,:,0] = np.cross(tmpCOS[:,:,1], tmpCOS[:,:,2], axis=1)
        self.rthigh = tmpCOS / np.expand_dims(np.linalg.norm(tmpCOS, axis=1), axis=1)

        # Left thigh coordinate system (Proximal = Wu)
        tmpCOS[:,:,1] = jointorigins.lhip-jointorigins.lknee
        tmp = np.cross( (jointorigins.lhip-markers['LKNEM']), (jointorigins.lhip-markers['LKNE']), axis=1)
        tmpCOS[:,:,2] = np.cross(tmp, tmpCOS[:,:,1], axis=1)
        tmpCOS[:,:,0] = np.cross(tmpCOS[:,:,1] , tmpCOS[:,:,2], axis=1)
        self.lthigh = tmpCOS / np.expand_dims(np.linalg.norm(tmpCOS, axis=1), axis=1)

        # Calculate shank coordinate sytems
        tmpCOS[:,:,2] = markers['RANK']-markers['RANKM']
        tmpCOS[:,:,0] = np.cross( (markers['RANK']-jointorigins.rknee), (markers['RANKM']-jointorigins.rknee), axis=1)
        tmpCOS[:,:,1] = np.cross(tmpCOS[:,:,2] , tmpCOS[:,:,0], axis=1)
        self.rshank = tmpCOS / np.expand_dims(np.linalg.norm(tmpCOS, axis=1), axis=1)

        tmpCOS[:,:,2] = markers['LANKM']-markers['LANK']
        tmpCOS[:,:,0] = np.cross( (markers['LANKM']-jointorigins.lknee), (markers['LANK']-jointorigins.lknee), axis=1)
        tmpCOS[:,:,1] = np.cross(tmpCOS[:,:,2] , tmpCOS[:,:,0], axis=1)
        self.lshank = tmpCOS / np.expand_dims(np.linalg.norm(tmpCOS, axis=1), axis=1)

        # Calculate foot coordinate sytems
        tmpCOS[:,:,0] = markers['RTOE']-markers['RHEEL']
        tmpCOS[:,:,1] = np.cross((markers['RCAL']-markers['RCALM']), tmpCOS[:,:,0], axis=1)
        tmpCOS[:,:,2] = np.cross(tmpCOS[:,:,0] , tmpCOS[:,:,1] ,axis=1)
        self.rfoot = tmpCOS / np.expand_dims(np.linalg.norm(tmpCOS, axis=1), axis=1)

        tmpCOS[:,:,0] = markers['LTOE']-markers['LHEEL']
        tmpCOS[:,:,1] = np.cross( tmpCOS[:,:,0] , (markers['LCAL']-markers['LCALM']),axis=1)
        tmpCOS[:,:,2] = np.cross( tmpCOS[:,:,0] , tmpCOS[:,:,1],axis=1)
        self.lfoot = tmpCOS / np.expand_dims(np.linalg.norm(tmpCOS, axis=1), axis=1)

        # Transform from 3x3xn matrices to nX4 quaternions
        def to_quat(x):
            r = R.from_matrix(x)
            quat = r.as_quat()
            quat[:, [1, 2, 3, 0]] = quat[:, [0, 1, 2, 3]]
            return quat

        self.pelvis = to_quat(self.pelvis)
        self.rthigh = to_quat(self.rthigh)
        self.lthigh = to_quat(self.lthigh)
        self.rshank = to_quat(self.rshank)
        self.lshank = to_quat(self.lshank)
        self.rfoot = to_quat(self.rfoot)
        self.lfoot = to_quat(self.lfoot)

def make_segcos(markers, joint_origins):
    return SegmentCoordinateSystem(markers, joint_origins)

class JointOrigins(object):
    def __init__(self, markers):
        # Calculate Ankle Joint Center
        self.rankle = markers['RANK'] + 0.5 * (markers['RANKM'] - markers['RANK'])
        self.lankle = markers['LANK'] + 0.5 * (markers['LANKM'] - markers['LANK'])
        # Calculate Knee Joint Center
        self.rknee = markers['RKNE'] + 0.5 * (markers['RKNEM'] - markers['RKNE'])
        self.lknee = markers['LKNE'] + 0.5 * (markers['LKNEM'] - markers['LKNE'])
        # Calculate Hip Joint Center
        # Auxiliary Hip Joint Center
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
        PelvisCOS = PelvisCOS / np.expand_dims(np.linalg.norm(PelvisCOS, axis=1), axis=1)

        self.rhip = ASI+PelvisCOS[:,:,0] * np.array([(-0.24*PelvisDepth-9.9)]).T \
                    +PelvisCOS[:,:,1] * np.array([(-0.16*PelvisWidth-0.04*LegLengthRight-7.1)]).T \
                    +PelvisCOS[:,:,2] * np.array([(0.28*PelvisDepth+0.16*PelvisWidth+7.9)]).T

        self.lhip = ASI+PelvisCOS[:,:,0] * np.array([(-0.24*PelvisDepth-9.9)]).T \
                    +PelvisCOS[:,:,1] * np.array([(-0.16*PelvisWidth-0.04*LegLengthLeft-7.1)]).T \
                    -PelvisCOS[:,:,2] * np.array([(0.28*PelvisDepth+0.16*PelvisWidth+7.9)]).T

def make_joint_origins(markers):
    return JointOrigins(markers)

def loadmat(filename):
    '''
    this function should be called instead of direct spio.loadmat
    as it cures the problem of not properly recovering python dictionaries
    from mat files. It calls the function check keys to cure all entries
    which are still mat-objects
    '''
    data = spio.loadmat(filename, struct_as_record=False, squeeze_me=True)
    return _check_keys(data)

def _check_keys(dict):
    '''
    checks if entries in dictionary are mat-objects. If yes
    todict is called to change them to nested dictionaries
    '''
    for key in dict:
        if isinstance(dict[key], spio.matlab.mio5_params.mat_struct):
            dict[key] = _todict(dict[key])
    return dict

def _todict(matobj):
    '''
    A recursive function which constructs from matobjects nested dictionaries
    '''
    dict = {}
    for strg in matobj._fieldnames:
        elem = matobj.__dict__[strg]
        if isinstance(elem, spio.matlab.mio5_params.mat_struct):
            dict[strg] = _todict(elem)
        else:
            dict[strg] = elem
    return dict