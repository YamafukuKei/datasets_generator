#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import random
import numpy as np
from math import *
from time import sleep

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
from gazebo_msgs.msg import *
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs

from visualization_msgs.msg import Marker

rospy.init_node("random_state_maker", anonymous=True)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
pub1 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
pub2 = rospy.Publisher('/posestamped_obj', PoseStamped, queue_size=1)
r = rospy.Rate(8) # hz

def transform_3D_point_to_new_frame(xyz_point_from_sensor, from_frame_id):
    to_frame = "kinect_rgb_optical_frame"

    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = from_frame_id
    pose_stamped.pose.orientation.w = 1.0
    pose_stamped.pose.position.x = xyz_point_from_sensor.position.x
    pose_stamped.pose.position.y = xyz_point_from_sensor.position.y
    pose_stamped.pose.position.z = xyz_point_from_sensor.position.z
    pose_stamped.pose.orientation.x = xyz_point_from_sensor.orientation.x
    pose_stamped.pose.orientation.y = xyz_point_from_sensor.orientation.y
    pose_stamped.pose.orientation.z = xyz_point_from_sensor.orientation.z
    pose_stamped.pose.orientation.w = xyz_point_from_sensor.orientation.w

    transform = tf_buffer.lookup_transform(to_frame,
                                                pose_stamped.header.frame_id,
                                                rospy.Time(0),
                                                rospy.Duration(1.0))
    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    return pose_transformed


def publish_orientation(pos, ps, quat):
    pass
    pos.pose.orientation.x = quat[0]
    pos.pose.orientation.y = quat[1]
    pos.pose.orientation.z = quat[2]
    pos.pose.orientation.w = quat[3]

    ## transform the frame of PointCloud
    cloud_from_camera = transform_3D_point_to_new_frame(pos.pose, "world")
    ps.pose.position.x = cloud_from_camera.pose.position.x
    ps.pose.position.y = cloud_from_camera.pose.position.y
    ps.pose.position.z = cloud_from_camera.pose.position.z

    euler = euler_from_quaternion((cloud_from_camera.pose.orientation.x, cloud_from_camera.pose.orientation.y, cloud_from_camera.pose.orientation.z, cloud_from_camera.pose.orientation.w))
    ps.pose.orientation.x = euler[0]
    ps.pose.orientation.y = euler[1]
    ps.pose.orientation.z = euler[2]
    ps.pose.orientation.w = 0

##    print(cloud_from_camera)
    pub1.publish(pos) ## world coordinate
    ps.header.stamp = rospy.Time.now()
    pub2.publish(ps)  ## camera coordinate

##    print "ROS_time(state_pub) :",ps.header.stamp
##    print_commands(pos)
    r.sleep()


def random_state_make():
    # TODO: Print here the commands
    # tf static transform
    print("now making random states of target object...")
    sleep(0.1)
##    marker_pub = rospy.Publisher("transferred_image_pixel", Marker, queue_size = 10)
    pos = ModelState()
    ps = PoseStamped()
    pos.model_name = "kobject"
    lamda_x = 0
    lamda_y = 0
    lamda_z = 0
    initial_pose_num = 200

    z_step = 1 ## 1
    y_step_num = 10 ## 10
    theta_step = 18 ##18

    quat = 0
    theta = 0
    pattern = 0
    count = 0
    flag = 0
    while not flag:
        if (count < initial_pose_num):
            pos.pose.position.x = random.uniform(-0.2, 0.0) ##(-0.2, 0.45)
            pos.pose.position.y = random.uniform(0.35, -0.35)
            pos.pose.position.z = random.uniform(0.1, 0.4)
            ro = random.uniform(-3.14, 3.14)
            pi = random.uniform(-3.14, 3.14)
            ya = random.uniform(-3.14, 3.14)
            quat = quaternion_from_euler(ro, pi, ya)
            count += 1

            publish_orientation(pos, ps, quat)

        else:
            pos.pose.position.x = -0.2
            pos.pose.position.y = 0
            pos.pose.position.z = 0.1
            for lamda_z in range(-10,11,z_step):
                lamda_z = lamda_z * 0.1
                print("lamda_z:{}".format(lamda_z))
                if(lamda_z == -1 or lamda_z == 1):
                    lamda_y = 0
                    lamda_x = 0
                    print("lamda_y:{}".format(lamda_y))
                    print("sign:0")
                    for theta in range(0, 360, theta_step):
                        theta = theta*2*np.pi/360
                        print("theta:{}".format(theta))
                        for pattern in range(3):
                            if pattern == 0:
                                quat_x = lamda_x*np.sin(theta/2.0)
                                quat_y = lamda_y*np.sin(theta/2.0)
                                quat_z = lamda_z*np.sin(theta/2.0)
                                quat_w = np.cos(theta/2.0)
                            elif pattern == 1:
                                quat_x = lamda_y*np.sin(theta/2.0)
                                quat_y = lamda_z*np.sin(theta/2.0)
                                quat_z = lamda_x*np.sin(theta/2.0)
                                quat_w = np.cos(theta/2.0)
                            elif pattern == 2:
                                quat_x = lamda_z*np.sin(theta/2.0)
                                quat_y = lamda_x*np.sin(theta/2.0)
                                quat_z = lamda_y*np.sin(theta/2.0)
                                quat_w = np.cos(theta/2.0)
                            print("pattern:{}".format(pattern))

                            quat = [quat_x, quat_y, quat_z, quat_w]

                            publish_orientation(pos, ps, quat)
                else:
                    min_lamda_y = -np.sqrt(1.0-(lamda_z)**2.0)*1000.0
                    max_lamda_y = np.sqrt(1.0-(lamda_z)**2.0)*1000.0
                    step = (max_lamda_y - min_lamda_y)/y_step_num
                    # print(min_lamda_y)
                    # print(max_lamda_y)
                    # print(step)
                    for lamda_y in range(int(min_lamda_y), int(max_lamda_y), int(step)):
                        lamda_y = lamda_y * 0.001
                        lamda_x = np.sqrt(1-(lamda_y**2)-(lamda_z**2))
                        print("lamda_y:{}".format(lamda_y))
                        for sign in range(2):
                            if sign == 0:
                                lamda_x = lamda_x
                            elif sign == 1:
                                lamda_x = -lamda_x
                            print("sign:{}".format(sign))
                            for theta in range(0, 360, theta_step):
                                theta = theta*2*np.pi/360
                                print("theta:{}".format(theta))
                                for pattern in range(3):
                                    ##print(lamda_x, lamda_y, lamda_z, theta)
                                    if pattern == 0:
                                        quat_x = lamda_x*np.sin(theta/2.0)
                                        quat_y = lamda_y*np.sin(theta/2.0)
                                        quat_z = lamda_z*np.sin(theta/2.0)
                                        quat_w = np.cos(theta/2.0)
                                    elif pattern == 1:
                                        quat_x = lamda_y*np.sin(theta/2.0)
                                        quat_y = lamda_z*np.sin(theta/2.0)
                                        quat_z = lamda_x*np.sin(theta/2.0)
                                        quat_w = np.cos(theta/2.0)
                                    elif pattern == 2:
                                        quat_x = lamda_z*np.sin(theta/2.0)
                                        quat_y = lamda_x*np.sin(theta/2.0)
                                        quat_z = lamda_y*np.sin(theta/2.0)
                                        quat_w = np.cos(theta/2.0)
                                    print("pattern:{}".format(pattern))

                                    quat = [quat_x, quat_y, quat_z, quat_w]

                                    publish_orientation(pos, ps, quat)

        if (lamda_z==1 and theta==2*np.pi-(theta_step*2*np.pi/360) and pattern==2):
            print("Finish")
            for i in range(10):
                ps.pose.orientation.w = 1
                ps.header.stamp = rospy.Time.now()
                pub2.publish(ps)  ## camera coordinate
                r.sleep()
                flag = 1



if __name__ == "__main__":
    try:
        random_state_make()

    except rospy.ROSInterruptException: pass
