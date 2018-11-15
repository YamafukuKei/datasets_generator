#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import h5py
import csv

##from tqdm import tqdm
##import time
##from scipy import interpolate

div_num = 50*50*50

f_hdf5 = h5py.File('../cloud_pos_ori_10000.hdf5','w')

n_data = 2

for i in range(n_data):
    f = open("./learn_data/tf_"+str(i+1)+".csv",'r')
    tf_csv = csv.reader(f)
    lines_tf = [e for e in tf_csv]
    tf = [0]*7
    for j in range(0,7):
        tf[j] = float(lines_tf[j][0])
##    print(tf)

    fi = open("./learn_data/voxel_"+str(i+1)+".csv",'r')
    voxel_csv = csv.reader(fi)
    lines_voxel = [e for e in voxel_csv]
    voxel = [0]*div_num
    for k in range(0,div_num):
        voxel[k] = float(lines_voxel[k][0])
##    print(len(lines_voxel))

    data_group = f_hdf5.create_group("data_"+str(i+1))

##    raw_traj_len = len(lines)-2
##    raw_w = np.zeros((7, raw_traj_len))
##    raw_configuration_space_traj = np.zeros((6, raw_traj_len))
##
##    for i in range(raw_traj_len):
##        raw = [float(e) for e in lines[2+i].split(',')]
##        raw_w[:, i] = raw[:7]
##        raw_configuration_space_traj[:, i] = raw[7:]
##
##    #-- Resampling to a 100 waypoints trajectory --#
##    t_raw = np.linspace(0, 1, raw_traj_len)
##    t = np.linspace(0, 1, 100)
##    workspace_traj = np.zeros((7, 100))
##    configuration_space_traj = np.zeros((6, 100))
##
##    for j in range(7):
##        workspace_traj[j,:] = interpolate.interp1d(t_raw, raw_w[j,:])(t)
##
##    for j in range(6):
##        configuration_space_traj[j,:] = interpolate.interp1d(t_raw, raw_configuration_space_traj[j,:])(t)
##
    data_group.create_dataset("voxel", data=tf)
    data_group.create_dataset("pos_ori", data=voxel)
