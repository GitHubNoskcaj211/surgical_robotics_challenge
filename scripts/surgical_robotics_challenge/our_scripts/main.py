#!/usr/bin/env python

from get_waypoints import GetPaths
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import math
import numpy as np
from PyKDL import Rotation
from scipy.spatial.transform import Rotation as Rotation2
import time

rospy.init_node("main")
get_paths_right = GetPaths('psm2')

def set_needle_position(msg):
    global needle
    needle = matrix_from_transform(msg)

def set_base_position(msg):
    global base
    base = matrix_from_transform(msg)

def quaternion_to_rotation_matrix(Q):
    r = Rotation2.from_quat([Q.x, Q.y, Q.z, Q.w])
    return r.as_dcm()

def matrix_from_transform(transform):
    rotation_matrix = quaternion_to_rotation_matrix(transform.transform.rotation)
    full_matrix = np.append(
        rotation_matrix,
        [
            [transform.transform.translation.x],
            [transform.transform.translation.y],
            [transform.transform.translation.z],
        ],
        axis=1,
    )
    full_matrix = np.append(full_matrix, [[0, 0, 0, 1]], axis=0)
    return full_matrix

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

def set_arm_position(msg):
    global end_effector
    if type(base) is np.ndarray:
        base_to_end_effector = matrix_from_transform(msg)
        end_effector = np.matmul(base, base_to_end_effector)

def find_error(current,desired):
    currentTranslation = current[0:3,3]
    desiredTranslation = desired[0:3,3]
    difference = desiredTranslation - currentTranslation
    position_error = math.sqrt(np.dot(difference,difference))

    currentRotation = current[0:3,0:3]
    desiredRotation = desired[0:3,0:3]
    errorRotation = np.matmul(desiredRotation,np.transpose(currentRotation))
    try:
        rotation_error = math.acos((np.trace(errorRotation) - 1) / 2)
    except Exception:
        rotation_error = 0.0
    print("Position Error: ",position_error)
    print("Rotation Error: ",rotation_error)
    return (position_error,rotation_error)

def grab_right():
    temp = JointState()
    temp.position = [0.0]
    servo_jaw_right_pub.publish(temp)
    time.sleep(10) # TODO

def release_right():
    temp = JointState()
    temp.position = [0.5]
    servo_jaw_right_pub.publish(temp)
    time.sleep(10) # TODO

def grab_needle_right():
    rate = rospy.Rate(0.1)
    i = 0
    while not rospy.is_shutdown():
        path = get_paths_right.grab_needle_right()

        if path != None and end_effector is not None:
            if i < len(path):
                (position_error,rotation_error) = find_error(end_effector,path[i])
            while i < len(path) and position_error < 0.05 and rotation_error < 0.05:
                i = i + 1
                if i < len(path):
                    (position_error,rotation_error) = find_error(end_effector,path[i])
            if i < len(path):
                servo_cp_right_pub.publish(np_mat_to_transform(path[i]))
            else:
                grab_right()
                return True
        rate.sleep()

def nav_entry(index):
    rate = rospy.Rate(0.08)
    i = 0
    while not rospy.is_shutdown():
        path = get_paths_right.entry_path(index)

        if path != None and end_effector is not None:
            if i < len(path):
                (position_error,rotation_error) = find_error(end_effector,path[i])
            while i < len(path) and position_error < 0.05 and rotation_error < 0.05:
                i = i + 1
                if i < len(path):
                    (position_error,rotation_error) = find_error(end_effector,path[i])
            if i < len(path):
                servo_cp_right_pub.publish(np_mat_to_transform(path[i]))
            else:
                return True
        rate.sleep()

def nav_exit(index):
    rate = rospy.Rate(0.08)
    i = 0
    while not rospy.is_shutdown():
        path = get_paths_right.exit_path(index)

        if path != None and end_effector is not None:
            if i < len(path):
                (position_error,rotation_error) = find_error(end_effector,path[i])
            while i < len(path) and position_error < 0.05 and rotation_error < 0.05:
                i = i + 1
                if i < len(path):
                    (position_error,rotation_error) = find_error(end_effector,path[i])
            if i < len(path):
                servo_cp_right_pub.publish(np_mat_to_transform(path[i]))
            else:
                return True
        rate.sleep()

needle = None
rospy.Subscriber('/CRTK/Needle/measured_cp', TransformStamped, set_needle_position, queue_size=1)

base = None
rospy.Subscriber('/CRTK/' + 'psm2' + '/T_b_t_w', TransformStamped, set_base_position, queue_size=1)

end_effector = None
rospy.Subscriber('/CRTK/' + 'psm2' + '/measured_cp', TransformStamped, set_arm_position, queue_size=1)
# rospy.Subscriber('/CRTK/' + 'psm2' + '/measured_jp', JointState, set_arm_joints, queue_size=1)

servo_cp_right_pub = rospy.Publisher('/CRTK/psm2/servo_cp', TransformStamped, queue_size=1)
# servo_jp_right_pub = rospy.Publisher('/CRTK/psm2/servo_jp', JointState, queue_size=1)
servo_jaw_right_pub = rospy.Publisher('/CRTK/psm2/jaw/servo_jp', JointState, queue_size=1)

print('starting needle grab')
grab_needle_right()
print('starting nav entry')
nav_entry(0)
print('starting nav exit')
nav_exit(0)
