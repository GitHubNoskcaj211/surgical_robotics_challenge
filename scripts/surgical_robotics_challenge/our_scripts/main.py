#!/usr/bin/env python

from get_waypoints import GetPaths
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import math
import numpy as np
from PyKDL import Rotation

rospy.init_node("main")
get_paths_right = GetPaths('psm2')

servo_cp_right_pub = rospy.Publisher('/CRTK/psm2/servo_cp', TransformStamped, queue_size=1)

def np_mat_to_transform(cp):
    trans = TransformStamped()
    trans.transform.translation.x = cp[0, 3]
    trans.transform.translation.y = cp[1, 3]
    trans.transform.translation.z = cp[2, 3]

    Quat = rot_mat_to_quat(cp)

    trans.transform.rotation.x = Quat[0]
    trans.transform.rotation.y = Quat[1]
    trans.transform.rotation.z = Quat[2]
    trans.transform.rotation.w = Quat[3]
    return trans

def rot_mat_to_quat(cp):
    R = Rotation(cp[0, 0], cp[0, 1], cp[0, 2],
                 cp[1, 0], cp[1, 1], cp[1, 2],
                 cp[2, 0], cp[2, 1], cp[2, 2])

    return R.GetQuaternion()

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    path = get_paths_right.grab_needle_right()

    if path != None:
        servo_cp_right_pub.publish(np_mat_to_transform(path[1]))
    rate.sleep()
