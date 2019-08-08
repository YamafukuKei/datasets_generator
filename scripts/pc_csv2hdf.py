#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import h5py
import csv
import argparse

## save point cloud as pointnet dataset


#### 3d show
##from mpl_toolkits.mplot3d import Axes3D
##import matplotlib.pyplot as plt

from tqdm import tqdm
##import time
##from scipy import interpolate
parser = argparse.ArgumentParser()
parser.add_argument('--number', '-n', default=4199, type=int, #default=-1
                    help='the number of files')

args = parser.parse_args()
n_data = args.number

f_hdf5 = h5py.File('../cloud_pos_ori_10000.hdf5','w')
print("start converting")
for i in tqdm(range(1,n_data+1)):
    with open("./learn_data/tf_"+str(i)+".csv",'r') as f:
        tf_csv = csv.reader(f)
        lines_tf = [e for e in tf_csv]
        tf = [0]*7
        for j in range(0,7):
            tf[j] = float(lines_tf[j][0])

##    point_x = []
##    point_y = []
##    point_z = []
##    fig = plt.figure()
##    ax = fig.add_subplot(111, projection='3d')
##    ax.set_xlabel("x")
##    ax.set_ylabel("y")
##    ax.set_zlabel("z")

    with open("./learn_data/normarized_pc"+str(i)+".csv",'r') as fi:
        pc_csv = csv.reader(fi)
        lines_pc = [g for g in pc_csv]
        pc = np.zeros((len(lines_pc), 3), dtype=float)
        for k in range(0,len(lines_pc)):
            pc[k][0] = float(lines_pc[k][0])
            pc[k][1] = float(lines_pc[k][1])
            pc[k][2] = float(lines_pc[k][2])

##        print(len(voxel))
##        for i in range(div):
##            for j in range(div):
##                for k2 in range(div):
##                    if(float(lines_voxel[(div*div*i) + (div*j) + (k2)][0]) == 1):
##                        point_x.append(k2)
##                        point_y.append(j)
##                        point_z.append(i)
##
##        ax.scatter(point_x, point_y, point_z)
##        plt.show()
##    print(voxel.count(1))

    data_group = f_hdf5.create_group("data_"+str(i))

    data_group.create_dataset("pointcloud", data=pc)
    data_group.create_dataset("pose", data=tf, compression= "lzf")

f.close()
print("finish !!")
