#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import rospy
import math
from scipy.spatial.transform import Rotation
import numpy as np

def matrix_from_waypoint(self, waypoint):
    rotation_matrix = self.quaternion_to_rotation_matrix(transform.transform.rotation)
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

def rotation_matrix_to_quaternion(self, matrix):
    r = Rotation.from_matrix(matrix)
    return r.as_quat()

def euler_to_rotation_matrix(self, E):
    r = Rotation.from_quat([Q.x, Q.y, Q.z, Q.w])
    return r.as_matrix()

def rotation_matrix_to_euler(self, matrix):
    r = Rotation.from_matrix(matrix)
    return r.as_euler("xyz")

# GetArmTransform -- given a wanted waypoint for some frame and transform from arm frame to that frame
def get_arm_waypoint(other_frame_waypoint, arm_to_other_frame):
    # waypoints are np arrays of [x,y,z,roll,pitch,yaw] in relation to the world
    arm_to__other_frame_matrix = matrix_from_transform(arm_to_other_frame)
    world_to_waypoint = matrix_from_transform(frame_transform)

class GetPaths():
    def __init__(self, arm_frame_id):
        self.world_to_needle = None
        rospy.Subscriber('/CRTK/Needle/measured_cp', TransformStamped, self.set_needle_position, queue_size=1)

        self.needle_to_needle_grab_right = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.needle_to_needle_tip = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        self.needle_to_needle_grab_left = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        self.world_to_arm = None
        rospy.Subscriber('/CRTK/' + arm_frame_id + '/measured_cp', TransformStamped, self.set_arm_position, queue_size=1)

        self.world_to_entrys = None
        rospy.Subscriber('/CRTK/Entry1/measured_cp', TransformStamped, self.set_entry1, queue_size=1)
        rospy.Subscriber('/CRTK/Entry2/measured_cp', TransformStamped, self.set_entry2, queue_size=1)
        rospy.Subscriber('/CRTK/Entry3/measured_cp', TransformStamped, self.set_entry3, queue_size=1)
        rospy.Subscriber('/CRTK/Entry4/measured_cp', TransformStamped, self.set_entry4, queue_size=1)

        self.world_to_exits = None
        rospy.Subscriber('/CRTK/Exit1/measured_cp', TransformStamped, self.set_exit1, queue_size=1)
        rospy.Subscriber('/CRTK/Exit2/measured_cp', TransformStamped, self.set_exit2, queue_size=1)
        rospy.Subscriber('/CRTK/Exit3/measured_cp', TransformStamped, self.set_exit3, queue_size=1)
        rospy.Subscriber('/CRTK/Exit4/measured_cp', TransformStamped, self.set_exit4, queue_size=1)

    def set_needle_position(self, msg):
        self.world_to_needle = self.matrix_from_transform(msg)

    def set_arm_position(self, msg):
        # TODO convert the frame to get the arm in the world frame
        self.world_to_arm = self.matrix_from_transform(msg)

    def set_entry1(self, msg):
        self.world_to_entrys[0] = self.matrix_from_transform(msg)

    def set_entry2(self, msg):
        self.world_to_entrys[1] = self.matrix_from_transform(msg)

    def set_entry3(self, msg):
        self.world_to_entrys[2] = self.matrix_from_transform(msg)

    def set_entry4(self, msg):
        self.world_to_entrys[3] = self.matrix_from_transform(msg)

    def set_exit1(self, msg):
        self.world_to_exits[0] = self.matrix_from_transform(msg)

    def set_exit2(self, msg):
        self.world_to_exits[1] = self.matrix_from_transform(msg)

    def set_exit3(self, msg):
        self.world_to_exits[2] = self.matrix_from_transform(msg)

    def set_exit4(self, msg):
        self.world_to_exits[3] = self.matrix_from_transform(msg)

    # path to grab the needle with the right arm
    def grab_needle_right(self):
        if type(self.needle) is np.ndarray:
            needle_grab = np.matmul(np.linalg.inv(self.needle_to_needle_grab_right), self.world_to_needle)
            needle_intermediate_to_needle = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-1],[0,0,0,1]])
            needle_intermediate = np.matmul(needle_intermediate_to_needle, self.world_to_needle)
            path = [needle_intermediate, needle_grab]
            return path
        else:
            return None

    # path to the entry of the hole with the specified index
    def entry_path(self, index):
        pass

    # path for the arm to go through the hole and poke the needle to the other side
    def exit_path(self, index):
        pass

    # return the arm to home (above the hole for the index)
    def arm_home_path(self, index):
        pass

    # path to grab the needle with the left arm
    def grab_needle_left(self):
        pass

    def quaternion_to_rotation_matrix(self, Q):
        r = Rotation.from_quat([Q.x, Q.y, Q.z, Q.w])
        return r.as_dcm()

    def matrix_from_transform(self, transform):
        rotation_matrix = self.quaternion_to_rotation_matrix(transform.transform.rotation)
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

if __name__ == '__main__':
    rospy.init_node("get_waypoints_nodes")
    get_paths_node = GetPaths('psm1')
    rospy.spin()
