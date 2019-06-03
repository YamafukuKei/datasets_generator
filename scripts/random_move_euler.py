#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import random
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from gazebo_msgs.msg import *
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
from time import sleep

rospy.init_node("random_state_maker", anonymous=True)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

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

def random_state_make():
    # TODO: Print here the commands
    # tf static transform
    print("now making random states of target object...")
    sleep(0.1)
    pub1 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pub2 = rospy.Publisher('/posestamped_obj', PoseStamped, queue_size=1)
##    marker_pub = rospy.Publisher("transferred_image_pixel", Marker, queue_size = 10)
    pos = ModelState()
    ps = PoseStamped()
    r = rospy.Rate(10) # hz
    pos.model_name = "kobject"
    while not rospy.is_shutdown():
        pos.pose.position.x = random.uniform(0.45, 0)
        pos.pose.position.y = random.uniform(0.3, -0.3)
        pos.pose.position.z = random.uniform(0.1, 0.5)
        ro = random.uniform(-3.14, 3.14)
        pi = random.uniform(-3.14, 3.14)
        ya = random.uniform(-3.14, 3.14)

        quat = quaternion_from_euler(ro, pi, ya)
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

##        print(cloud_from_camera)
        pub1.publish(pos) ## world coordinate
        ps.header.stamp = rospy.Time.now()
        pub2.publish(ps)  ## camera coordinate

        print "ROS_time(state_pub) :",ps.header.stamp
##        print_commands(pos)
        r.sleep()

if __name__ == "__main__":
    try:
        random_state_make()

    except rospy.ROSInterruptException: pass
