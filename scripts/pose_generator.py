#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import random
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, PointStamped, Vector3, Quaternion
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from gazebo_msgs.msg import *
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker

import numpy as np


tf_buffer = tf2_ros.Buffer()
tf_listner = tf2_ros.TransformListener(tf_buffer)

a = 0
count = 0
switch_num = 40

def transform_3D_point_to_new_frame(xyz_point, from_frame_id):
    to_frame = "kinect_rgb_optical_frame"

    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = from_frame_id
    pose_stamped.pose.orientation.w = 1.0
    pose_stamped.pose.position.x = xyz_point.position.x
    pose_stamped.pose.position.y = xyz_point.position.y
    pose_stamped.pose.position.z = xyz_point.position.z
    pose_stamped.pose.orientation.x = xyz_point.orientation.x
    pose_stamped.pose.orientation.y = xyz_point.orientation.y
    pose_stamped.pose.orientation.z = xyz_point.orientation.z
    pose_stamped.pose.orientation.w = xyz_point.orientation.w

    transform = tf_buffer.lookup_transform(to_frame,
                                                pose_stamped.header.frame_id,
                                                rospy.Time(0),
                                                rospy.Duration(1.0))
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    return pose_transformed

def rotM(p):
    # 回転行列を計算する
    px = p[0]
    py = p[1]
    pz = p[2]

    # 物体座標系の 1->2->3 軸で回転させる
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(px), np.sin(px)],
                   [0, -np.sin(px), np.cos(px)]])
    Ry = np.array([[np.cos(py), 0, -np.sin(py)],
                   [0, 1, 0],
                   [np.sin(py), 0, np.cos(py)]])
    Rz = np.array([[np.cos(pz), np.sin(pz), 0],
                   [-np.sin(pz), np.cos(pz), 0],
                   [0, 0, 1]])
    R = Rz.dot(Ry).dot(Rx)

    # 物体座標系の 3->2->1 軸で回転させる
    # Rx = np.array([[1, 0, 0],
    #                [0, np.cos(px), np.sin(px)],
    #                [0, -np.sin(px), np.cos(px)]])
    # Ry = np.array([[np.cos(py), 0, -np.sin(py)],
    #                [0, 1, 0],
    #                [np.sin(py), 0, np.cos(py)]])
    # Rz = np.array([[np.cos(pz), np.sin(pz), 0],
    #                [-np.sin(pz), np.cos(pz), 0],
    #                [0, 0, 1]])
    # R = Rx.dot(Ry).dot(Rz)

    # 空間座標系の 1->2->3 軸で回転させる
    # Rx = np.array([[1, 0, 0],
    #                [0, np.cos(px), -np.sin(px)],
    #                [0, np.sin(px), np.cos(px)]])
    # Ry = np.array([[np.cos(py), 0, -np.sin(py)],
    #                [0, 1, 0],
    #                [np.sin(py), 0, np.cos(py)]])
    # Rz = np.array([[np.cos(pz), np.sin(pz), 0],
    #                [-np.sin(pz), np.cos(pz), 0],
    #                [0, 0, 1]])
    # R = Rx.dot(Ry).dot(Rz)

    # 空間座標系の 3->2->1 軸で回転させる
    # Rx = np.array([[1, 0, 0],
    #                [0, np.cos(px), -np.sin(px)],
    #                [0, np.sin(px), np.cos(px)]])
    # Ry = np.array([[np.cos(py), 0, -np.sin(py)],
    #                [0, 1, 0],
    #                [np.sin(py), 0, np.cos(py)]])
    # Rz = np.array([[np.cos(pz), np.sin(pz), 0],
    #                [-np.sin(pz), np.cos(pz), 0],
    #                [0, 0, 1]])
    # R = Rz.dot(Ry).dot(Rx)
    return R

def random_state_make():
    global a
    global count
    global switch_num

    # TODO: Print here the commands
    # tf static transform
    print("now making random states of target object...")
    pub1 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pub2 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pub3 = rospy.Publisher('/posestamped_obj', PoseStamped, queue_size=1)
    pub4 = rospy.Publisher('/principal', PoseStamped, queue_size=1)
##    marker_pub = rospy.Publisher("transferred_image_pixel", Marker, queue_size = 10)
    pos = ModelState()
    pos_obstacle = ModelState()
    ps = PoseStamped()
    principal = PoseStamped()
    r = rospy.Rate(10) # hz
    pos.model_name = "kobject"
    while not rospy.is_shutdown():
##        pos.pose.position.x = random.uniform(0.5, -0.5)
##        pos.pose.position.y = random.uniform(0.4, -0.4)
##        pos.pose.position.z = random.uniform(0.1, 0.5)

##        pos.pose.position.x = 0.2
##        pos.pose.position.y = 0
##        pos.pose.position.z = 0.3
        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.position.z = 0


##        ro = random.uniform(-3.14, 3.14)
##        pi = random.uniform(-3.14, 3.14)
##        ya = random.uniform(-3.14, 3.14)
        ro = 1.57
        pi = 0
        ya = 0

        ## transform the frame of PointCloud
        if (count % switch_num != 0 or a == 0 ):
            cloud_from_camera = transform_3D_point_to_new_frame(pos.pose, "world")
        ps.pose.position.x = cloud_from_camera.pose.position.x
        ps.pose.position.y = cloud_from_camera.pose.position.y
        ps.pose.position.z = cloud_from_camera.pose.position.z
        ps.pose.orientation.x = cloud_from_camera.pose.orientation.x
        ps.pose.orientation.y = cloud_from_camera.pose.orientation.y
        ps.pose.orientation.z = cloud_from_camera.pose.orientation.z
        ps.pose.orientation.w = cloud_from_camera.pose.orientation.w

        ## calculate transformed principal vector
        princi_vec = np.array([0.065, 0.1, 0.25])
        p = np.array([ro, pi, ya])
        R = rotM(p)
        A = R.T
        principal_new = np.dot(A, princi_vec)
        principal.pose.position.x = principal_new[0]
        principal.pose.position.y = principal_new[1]
        principal.pose.position.z = principal_new[2]
        new_frame_principal = transform_3D_point_to_new_frame(principal.pose, "world")
        principal.pose.position.x = new_frame_principal.pose.position.x
        principal.pose.position.y = new_frame_principal.pose.position.y
        principal.pose.position.z = new_frame_principal.pose.position.z
        ##print(principal.pose.position)


        quat = quaternion_from_euler(ro, pi, ya)
        print(quat)
        pos.pose.orientation.x = quat[0]
        pos.pose.orientation.y = quat[1]
        pos.pose.orientation.z = quat[2]
        pos.pose.orientation.w = quat[3]

        if (count % switch_num == 0):
            if (count == 0):
                ro = 0
                pi = 0
                ya = 0
                quat_obs = quaternion_from_euler(ro, pi, ya)
                pos_obstacle.pose.orientation.x = quat_obs[0]
                pos_obstacle.pose.orientation.y = quat_obs[1]
                pos_obstacle.pose.orientation.z = quat_obs[2]
                pos_obstacle.pose.orientation.w = quat_obs[3]
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = -0.3

            if (count == switch_num * 1):
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = 0.13
                pos_obstacle.pose.position.z = 0.3
                pos_obstacle.model_name = "obstacle"
            if (count == switch_num * 2):
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = 0.45
                pos_obstacle.model_name = "obstacle"
            if (count == switch_num * 3):
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = -0.13
                pos_obstacle.pose.position.z = 0.3
                pos_obstacle.model_name = "obstacle"
            if (count == switch_num * 4):
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = 0.17
                pos_obstacle.model_name = "obstacle"

            if (count == switch_num * 5):
                pos_obstacle.pose.position.x = 0.6
                pos_obstacle.pose.position.y = 0.13
                pos_obstacle.pose.position.z = 0.4
                pos_obstacle.model_name = "obstacle"
            if (count == switch_num * 6):
                pos_obstacle.pose.position.x = 0.6
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = 0.53
                pos_obstacle.model_name = "obstacle"
            if (count == switch_num * 7):
                pos_obstacle.pose.position.x = 0.6
                pos_obstacle.pose.position.y = -0.13
                pos_obstacle.pose.position.z = 0.4
                pos_obstacle.model_name = "obstacle"
            if (count == switch_num * 8):
                pos_obstacle.pose.position.x = 0.6
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = 0.27
                pos_obstacle.model_name = "obstacle"

            if (count == switch_num * 9):
                pos_obstacle.pose.position.x = 0.07
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = 0.3
                pos_obstacle.model_name = "obstacle"

            if (count == switch_num * 10):
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = -0.3
                pos_obstacle.model_name = "obstacle"

            if (count == switch_num * 11):
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = 0.3
                pos_obstacle.model_name = "obstacle_small"

            if (count == switch_num * 14):
                pos_obstacle.pose.position.x = 0.2
                pos_obstacle.pose.position.y = 0
                pos_obstacle.pose.position.z = -0.3
                pos_obstacle.model_name = "obstacle_small"
                count = -1 * switch_num

            pub2.publish(pos_obstacle) ## obstacle cloud

        if (count % switch_num != 0):
            pub1.publish(pos) ## world coordinate object cloud
        ps.header.stamp = rospy.Time.now()
        pub3.publish(ps)  ## camera coordinate object tf
        ps.header.stamp = rospy.Time.now()
        pub4.publish(principal)  ## camera coordinate principal tf


        print "ROS_time(state_pub) :",ps.header.stamp
##        print_commands(pos)
        r.sleep()
        count += 1
        a += 1

if __name__ == "__main__":
    rospy.init_node("pose_generator", anonymous=True)
    try:
        random_state_make()

    except rospy.ROSInterruptException: pass
