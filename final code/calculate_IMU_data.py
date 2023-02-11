import glob
import numpy as np
from defs import loadmat, make_joint_origins, make_segcos, make_position, make_orientation, make_accelerometer, make_gyroscope
########################################################################################################################

########################################################################################################################
input_path = '/opt/DATA/mat/'
outputs_path = '/opt/DATA/npy/IMU/'

# markers necessary for joint angle calculation
markers_needed1 = ('RANK', 'RANKM', 'LANK', 'LANKM',
                  'RKNE', 'RKNEM', 'LKNE', 'LKNEM',
                  'RPSI', 'LPSI', 'RASI', 'LASI',
                  'RTOE', 'RHEEL', 'RCAL', 'RCALM',
                  'LTOE', 'LHEEL', 'LCAL', 'LCALM')

markers_needed2 = ('RANK', 'RANKM', 'LANK', 'LANKM',
                  'RKNE', 'RKNEM', 'LKNE', 'LKNEM',
                  'RPSI', 'LPSI', 'RASI', 'LASI',
                  'RTOE', 'RHEE', 'RCALL', 'RCALM',
                  'LTOE', 'LHEE', 'LCALL', 'LCALM')

# input files
files = glob.glob(input_path + '*.mat')

for file in files:
    IMU = dict()
    name = file.split('/')[-1].split('.')[0]
    markers = loadmat(file)

    # check if all necessary markers are available
    if all(marker in markers['markers'] for marker in markers_needed1):
        print('All markers available.')

    elif all(marker in markers['markers'] for marker in markers_needed2):
        print('Rename markers.')
        markers['markers']['RCAL'] = markers['markers']['RCALL']
        markers['markers']['LCAL'] = markers['markers']['LCALL']
        markers['markers']['RHEEL'] = markers['markers']['RHEE']
        markers['markers']['LHEEL'] = markers['markers']['LHEE']

    else:
        print(name)
        print('Not enough markers.')
        continue

    # Calculate kinematic quantities
    jointorigins = make_joint_origins(markers['markers'])
    segmentcos = make_segcos(markers['markers'], jointorigins)

    # Calculate synthetic IMU data
    position = make_position(markers['markers'], jointorigins, segmentcos)
    orientation = make_orientation(segmentcos)
    IMU['acc'] = make_accelerometer(position, orientation)
    IMU['gyr'] = make_gyroscope(orientation)

    np.save(outputs_path + name, IMU)
print('Finished')