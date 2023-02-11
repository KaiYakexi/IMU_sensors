import numpy as np
import os
from scipy.linalg import block_diag
import warnings

base_path = "/Users/dominikfohrmann/MT_data/"

# specify which markers are needed for the analysis
markers_needed = ['LASI', 'LPSI', 'RPSI', 'RASI',
                  'RTRO', 'RKNE', 'RKNEM', 'RTIB',
                  'RANK', 'RANKM', 'RCALL', 'RCALM', 'RHEE', 'RLMT5', 'RLMT1', 'RTOE',
                  'LTRO', 'LKNE', 'LKNEM', 'LTIB',
                  'LANK', 'LANKM', 'LCALL', 'LCALM', 'LHEE', 'LLMT5', 'LLMT1', 'LTOE']


def read_data_structure(data_root):
    data_structure = os.listdir(data_root)
    files = []
    for folder in data_structure:
        if not folder.startswith("."):
            content = os.listdir(os.path.join(data_root, folder))
            for c in content:
                if c.endswith(".c3d"):
                    files.append(os.path.join(data_root, folder, c))
    return files


def normalize_length_vector(v, l2):
    l1 = len(v)
    xp = np.linspace(0, l2 - 1, l1)
    v_norm = np.zeros(l2)
    for i in range(0, l2):
        v_norm[i] = np.interp(i, xp, v)
    return v_norm


def translate_markers(markers, x, y, z):
    for m in markers:
        markers[m].T[0] += x
        markers[m].T[1] += y
        markers[m].T[2] += z

    return True


def translate_corners(corners, x, y, z):
    # FP1 all corner coordinates
    corners[0][0::3] += x
    corners[0][1::3] += y
    corners[0][2::3] += z
    # FP2 all corner coordinates
    corners[1][0::3] += x
    corners[1][1::3] += y
    corners[1][2::3] += z

    return True


def mirror_corners_at_global_xz(orig_crns):
    crns_new = orig_crns.copy()

    mat33 = np.array([[1, 0, 0],
                      [0, -1, 0],
                      [0, 0, 1]])

    mat1212 = block_diag(mat33, mat33, mat33, mat33)
    crns_new = np.matmul(crns_new, mat1212)

    return crns_new


def swap_force_plates(corners, signals):
    # swap FP1 and Fp2 data (signals and corners)
    fp1_signals = signals[:6].copy()
    fp2_signals = signals[6:].copy()
    signals[:6] = fp2_signals
    signals[6:] = fp1_signals

    fp1_corners = corners[0].copy()
    fp2_corners = corners[1].copy()
    corners[0] = fp2_corners
    corners[1] = fp1_corners

    return True


def swap_fp2_corners(corners):
    # if we rotate FP2 for files L2 and R1 we need to swap the corners too
    # C1 <-> C3
    # C2 <-> C4

    i = 1
    corners[i][0], corners[i][6] = corners[i][6], corners[i][0]
    corners[i][1], corners[i][7] = corners[i][7], corners[i][1]

    corners[i][3], corners[i][9] = corners[i][9], corners[i][3]
    corners[i][4], corners[i][10] = corners[i][10], corners[i][4]

    return True


def rotate_fp2_signals(fp_sig):
    fp_sig[6] *= -1
    fp_sig[7] *= -1
    fp_sig[9] *= -1
    fp_sig[10] *= -1

    return True


def rotate_markers_around_global_z(markers, deg):
    rad = np.deg2rad(deg)
    c = np.cos(rad)
    s = np.sin(rad)
    mat33 = np.array([[c, -s, 0],
                      [s, c, 0],
                      [0, 0, 1]])

    for m in markers.keys():
        traj_3n = markers[m].T  # trajectory in the form 3 by n
        traj_new = np.matmul(mat33, traj_3n).T
        markers[m] = traj_new

    return True


def rotate_corners_around_global_z(corners, deg):
    rad = np.deg2rad(deg)
    c = np.cos(rad)
    s = np.sin(rad)
    mat33 = np.array([[c, -s, 0],
                      [s, c, 0],
                      [0, 0, 1]])

    mat1212 = block_diag(mat33, mat33, mat33, mat33)
    for c in range(corners.shape[0]):
        corners[c] = np.matmul(mat1212, corners[c].T).T

    return True


def mirror_markers_at_global_xz(markers):
    mat33 = np.array([[1, 0, 0],
                      [0, -1, 0],
                      [0, 0, 1]])

    for m in markers.keys():
        traj_3n = markers[m].T  # trajectory in the form 3 by n
        traj_new = np.matmul(mat33, traj_3n).T
        markers[m] = traj_new

    return True


def swap_marker_labels_left_right(markers):
    # if we mirror the marker trajectories, we also have to mirror the the marker labels,
    # e.g. RASI becomes LASI etc...
    new_markers = {}
    for m in markers.keys():
        if m.startswith('L'):
            new_key = m.replace("L", "R", 1)
            new_markers[new_key] = markers[m]
        elif m.startswith('R'):
            new_key = m.replace("R", "L", 1)
            new_markers[new_key] = markers[m]
        else:
            new_key = m
            new_markers[new_key] = markers[m]

    return new_markers


def mirror_signals_at_global_xz(fp_sig):
    # mirror force plate signals at global x-z-plane
    # For FP1
    fp_sig[0] *= -1  # Fx
    fp_sig[4] *= -1  # My
    fp_sig[5] *= -1  # Mz

    # For FP2
    fp_sig[6] *= -1  # Fx
    fp_sig[10] *= -1  # My
    fp_sig[11] *= -1  # Mz

    return True


def get_fp_corners(acq):
    # get force plate corners from btkAcquisition object in a numpy array
    # returns the force plate corners and origin coordinates as numpy array (n x 12) with n = number of force plates
    # TODO: make more robust for n != 2 number of force plates
    meta = acq.GetMetaData()
    for m in range(0, meta.GetChildNumber()):
        # print(m, meta.GetChild(m).GetLabel())
        if "FORCE_PLATFORM" in meta.GetChild(m).GetLabel():
            force_platform = meta.GetChild(m)
    # TODO: write extra function for that part:
    for fp in range(0, force_platform.GetChildNumber()):
        # print(fp, force_platform.GetChild(fp).GetLabel())
        if "ORIGIN" in force_platform.GetChild(fp).GetLabel():
            o = force_platform.GetChild(fp)
        elif "CORNERS" in force_platform.GetChild(fp).GetLabel():
            c = force_platform.GetChild(fp)

    # corners is a matrix(4, 3) with the coordinates of the 4 corners expressed in the global (laboratory) frame
    # origin is a vector(1, 3) containing the location of the sensor related to the center of the
    # ...working surface and expressed in the global frame

    fp_corners = np.asarray(c.GetInfo().ToDouble()).reshape((2, 12))
    # fp_origin = np.asarray(o.GetInfo().ToDouble()).reshape(2, 3)

    return fp_corners


def set_fp_corners(corners, acq):
    # write force plate corners from numpy array to btkAcquisition object
    c_reshaped = corners.flatten()
    meta = acq.GetMetaData()
    # look for index of force platform meta child
    for m in range(meta.GetChildNumber()):
        if "FORCE_PLATFORM" in meta.GetChild(m).GetLabel():
            fp_meta = meta.GetChild(m)
            break

    for fpm in range(fp_meta.GetChildNumber()):
        if "CORNERS" in fp_meta.GetChild(fpm).GetLabel():
            c_info = fp_meta.GetChild(fpm).GetInfo()
            break
    c_info.SetValues(c_reshaped)
    c_info.SetDimensions((3, 4, 2))
    return True


def get_fp_signals(acq):
    # get force plate signals from btkAcquisition object in a numpy array
    # TODO: Read meta data to check which channel is which
    meta = acq.GetMetaData()
    # look for index of force platform meta child
    for m in range(meta.GetChildNumber()):
        if "FORCE_PLATFORM" in meta.GetChild(m).GetLabel():
            fp_meta = meta.GetChild(m)
            break

    for fpm in range(fp_meta.GetChildNumber()):
        if "CHANNEL" in fp_meta.GetChild(fpm).GetLabel():
            channels = fp_meta.GetChild(fpm).GetInfo().ToInt()
            break

    a = acq.GetAnalogs()
    n_chan = a.GetItemNumber()
    n_samples = acq.GetAnalogFrameNumber()

    fp_sig = np.zeros((n_samples, n_chan)).T
    for grw in range(n_chan):
        fp_sig[:][grw] = a.GetItem(grw).GetValues().reshape(n_samples)

    return fp_sig


def set_fp_signals(fp_sig, acq):
    # write force plate signals from numpy array to btkAcquisition object
    a = acq.GetAnalogs()
    n = acq.GetAnalogFrameNumber()
    for grw in range(a.GetItemNumber()):
        a.GetItem(grw).SetValues(fp_sig[:][grw].reshape((n, 1)))

    return True


def get_markers(acq, markers_needed=None):
    # get marker data from btkAcquisition object in a dictionary
    markers = {}
    points = acq.GetPoints()
    for p in range(points.GetItemNumber()):
        label = points.GetItem(p).GetLabel()
        # print(label)
        data = points.GetItem(p).GetData().GetValues()
        if markers_needed is None:
            markers[label] = data
        else:
            if label in markers_needed:
                markers[label] = data

    return markers


def set_markers(markers, acq):
    # write marker data from dictionary to btkAcquisition object
    points = acq.GetPoints()
    for p in range(points.GetItemNumber()):
        label = points.GetItem(p).GetLabel()
        # print(label)
        if label in markers.keys():
            points.GetItem(p).SetValues(markers[label])

    return True


def get_initial_contact(fp_z, f):
    z = fp_z * -1  # convert to positive values
    threshold = 20
    ic_frame = 0
    ic = 1
    window = int(f / 10)  # window length is depending on the sample frequency = f/10
    while ic_frame == 0:
        if z[ic] >= threshold:
            if np.min(z[ic:(ic + window)]) > threshold:
                ic_frame = ic
            else:
                ic += 1
        else:
            ic += 1

    return ic_frame


def get_toe_off(fp_z, f):
    z = np.flip(fp_z * -1)  # convert to positive values and flip
    l = len(z)
    threshold = 20
    to_frame = 0
    ho = 0
    window = int(f / 10)  # window length is depending on the sample frequency = f/10
    if np.min(z[0:window]) > threshold:
        warnings.warn("No toe off found. Taking the last frame instead.")
        to_frame = -1
    else:
        while to_frame == 0:
            if z[ho] >= threshold:
                if np.min(z[ho:(ho + window)]) > threshold:
                    to_frame = ho
                else:
                    ho += 1
            else:
                ho += 1

    return l - to_frame + 1


def normalize_length(inp, l2):
    if inp.ndim == 1:
        return normalize_length_vector(inp, l2)
    elif inp.ndim == 2:
        # TODO: finish...
        l1 = len(inp)
        xp = np.linspace(0, l2 - 1, l1)
        v_norm = np.zeros(l2)
        for n in range(inp.shape[1]):
            for i in range(0, l2):
                v_norm[i] = np.interp(i, xp, inp)
    return v_norm


def resample_kinetic_data(fp_sig, f_actual, f_desired):
    if f_actual < f_desired:
        raise NotImplementedError("Converting to a higher frequency not implemented yet.")
    elif f_actual == f_desired:
        fp_sig_new = fp_sig
    elif f_actual % f_desired == 0:
        ratio = int(f_actual / f_desired)

        fp_sig_new = np.zeros((fp_sig.shape[0], int(fp_sig.shape[1] / ratio)))
        for f in range(len(fp_sig)):
            fp_sig_new[f] = fp_sig[f][0::ratio]

    else:
        l = fp_sig.shape[1]
        new_l = int(l * f_desired / f_actual)
        fp_sig_new = np.zeros((12, new_l))
        for f in range(len(fp_sig)):
            fp_sig_new[f] = normalize_length_vector(fp_sig[f], new_l)

    return fp_sig_new


def resample_kinematic_data(markers, f_actual, f_desired):
    if f_actual == f_desired:
        new_markers = markers
    else:
        new_markers = {}
        l = markers['LTOE'].shape[0]
        new_l = int(l * f_desired / f_actual)
        for m in markers.keys():
            traj3n = markers[m].T
            new_traj = np.zeros((3, new_l))
            for t in range(traj3n.shape[0]):
                new_traj[t] = normalize_length_vector(traj3n[t], new_l)
            new_markers[m] = new_traj.T

    return new_markers


def get_rmse(signal, mean_signal):

    n = signal.shape[0]
    rmse = np.power(np.sum(np.power((mean_signal - signal), 2))/n, 0.5)

    return rmse
