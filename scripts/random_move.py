#! /usr/bin/env python
# -*- coding: utf-8 -*-
# based on https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/basic_controls.py
# and https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/menu.py

import sys
import random
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from gazebo_msgs.msg import *

##def print_commands(pose_camera, pose):
##    from_frame = "kinect"
##    to_frame = "kobject"
##
##    decimals = 4
##    p = pose.position
##    o = pose.orientation
##
##    p_camera = pose_camera.pose.position
##    o_camera = pose_camera.pose.orientation
##
##    print "Static transform publisher command (with quaternion):"
##    ori_q = str(round(o.x, decimals)) + " " + str(round(o.y, decimals)) + " " + str(round(o.z, decimals)) + " " + str(round(o.w, decimals))
##    static_tf_cmd = common_part + pos_part + " " + ori_q + " " + from_frame + " " + to_frame + " 50"
##    print "  " + static_tf_cmd
##    print
##
##    print "Roslaunch line of static transform publisher (rpy):"
##    node_name = "from_" + from_frame + "_to_" + to_frame + "_static_tf"
##    roslaunch_part = '  <node name="' + node_name + '" pkg="tf" type="static_transform_publisher" args="' +\
##                     pos_part + " " + ori_part + " " + from_frame + " " + to_frame + " 50" + '" />'
##    print roslaunch_part
##    print
##
##                     pos_tf_part + " " + ori_tf_part + " " + "kinect" + " " + to_frame + " 50" + '" />'
##    print roslaunch_part
##    print
##
##    print "URDF format:"  # <origin xyz="0.04149 -0.01221 0.001" rpy="0 0 0" />
##    print '  <origin xyz="' + pos_part + '" rpy="' + ori_part + '" />'
##    print "\n---------------------------"
##

def random_state_make():
    # TODO: Print here the commands
    # tf static transform

    pub1 = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    pub2 = rospy.Publisher('/posestamped_obj', PoseStamped, queue_size=1)
    pos = ModelState()
    ps = PoseStamped()
    r = rospy.Rate(20) # 10hz
    pos.model_name = "kobject"
    while not rospy.is_shutdown():
        pos.pose.position.x = random.uniform(0.5, -0.5)
        pos.pose.position.y = random.uniform(0.4, -0.4)
        pos.pose.position.z = random.uniform(0.1, 0.5)
        ro = random.uniform(-3.14, 3.14)
        pi = random.uniform(-3.14, 3.14)
        ya = random.uniform(-3.14, 3.14)
        quat = quaternion_from_euler(ro, pi, ya)
        pos.pose.orientation.x = quat[0]
        pos.pose.orientation.y = quat[1]
        pos.pose.orientation.z = quat[2]
        pos.pose.orientation.w = quat[3]

        ps.pose = pos.pose
        ps.header.frame_id = "kobject"
        ps.header.stamp = rospy.Time.now()

##        print "ROS_time(state_pub) :",ps.header.stamp

        pub1.publish(pos)
        pub2.publish(ps)

##        print_commands(pos)
        r.sleep()

if __name__ == "__main__":
    rospy.init_node("random_state_maker", anonymous=True)
    try:
        random_state_make()

    except rospy.ROSInterruptException: pass
