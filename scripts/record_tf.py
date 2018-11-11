#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
from gazebo_msgs.srv import SetModelState

def callback(pose):
    print(pose)

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('record_centroid', anonymous=True)

    ps = PoseStamped()
    rospy.Subscriber('/posestamped_obj', PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
