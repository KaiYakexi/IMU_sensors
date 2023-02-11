#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
"""

"""

import btk
import df_functions as df
import matplotlib.pyplot as plt
import scipy
import scipy.signal
import scipy.linalg
import df_functions as df
import argparse
import glob
import os
import shutil

def get_acq(name):
    reader = btk.btkAcquisitionFileReader()
    reader.SetFilename(name)
    reader.Update()
    return reader.GetOutput()
# get marker data from btkAcquisition object in a dictionary
def get_markers(acq):
    return df.get_markers(acq)
#plot markers
def plot_marker(name, markers):
    for label, values in markers.items():
        fig,(ax1,ax2,ax3) = plt.subplots(3,sharex = True)
        plt.suptitle(label)
        ax1.plot(values[:,0])
        ax1.set_title("X-axis")
        ax2.plot(values[:,1])
        ax2.set_title("Y-axis")
        ax3.plot(values[:,2])
        ax3.set_title("Z-axis")
        label = label.strip('*')
        plt.savefig(name + "-" + label + '.png')

class Markers(object):
    def __init__(self, markers):
        self.__dict__ = markers
