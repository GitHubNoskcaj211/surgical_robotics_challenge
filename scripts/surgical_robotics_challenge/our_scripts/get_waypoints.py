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
        self.average = np.full((100,4,4), None, dtype=np.float32)

        self.needle = None
        rospy.Subscriber('/CRTK/Needle/measured_cp', TransformStamped, self.set_needle_position, queue_size=1)

        self.base = None
        rospy.Subscriber('/CRTK/' + arm_frame_id + '/T_b_t_w', TransformStamped, self.set_base_position, queue_size=1)

        # self.needle_to_needle_grab_right = np.array([[-3.5784909e-01,  9.3377423e-01,  3.3670664e-03,  9.2655444e-01],
        #                                              [ 9.2014331e-01,  3.5323501e-01, -1.6899803e-01, -1.1542608e+00],
        #                                              [-1.5899545e-01, -5.7377659e-02, -9.8561090e-01, -3.5336194e+00],
        #                                              [ 0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00]])

        self.needle_to_needle_grab_right = np.array([[0,  1,  0,  -0.1],
                                                     [ 1,  0, 0, 0.01],
                                                     [0, 0, -1, -.03],
                                                     [ 0,0,0,1]])

        self.needle_to_needle_tip = np.array([[0.866,0,-0.5,0.05],[-0.5,0,-0.866,.0866],[0,1,0,0],[0,0,0,1]])
        self.needle_to_needle_grab_left = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        self.end_effector = None
        rospy.Subscriber('/CRTK/' + arm_frame_id + '/measured_cp', TransformStamped, self.set_arm_position, queue_size=1)

        self.world_to_entrys = [None] * 4
        rospy.Subscriber('/CRTK/Entry1/measured_cp', TransformStamped, self.set_entry1, queue_size=1)
        rospy.Subscriber('/CRTK/Entry2/measured_cp', TransformStamped, self.set_entry2, queue_size=1)
        rospy.Subscriber('/CRTK/Entry3/measured_cp', TransformStamped, self.set_entry3, queue_size=1)
        rospy.Subscriber('/CRTK/Entry4/measured_cp', TransformStamped, self.set_entry4, queue_size=1)

        self.world_to_exits = [None] * 4
        rospy.Subscriber('/CRTK/Exit1/measured_cp', TransformStamped, self.set_exit1, queue_size=1)
        rospy.Subscriber('/CRTK/Exit2/measured_cp', TransformStamped, self.set_exit2, queue_size=1)
        rospy.Subscriber('/CRTK/Exit3/measured_cp', TransformStamped, self.set_exit3, queue_size=1)
        rospy.Subscriber('/CRTK/Exit4/measured_cp', TransformStamped, self.set_exit4, queue_size=1)

    def set_needle_position(self, msg):
        self.needle = self.matrix_from_transform(msg)

    def set_base_position(self, msg):
        self.base = self.matrix_from_transform(msg)

    def set_arm_position(self, msg):
        if type(self.base) is np.ndarray:
            base_to_end_effector = self.matrix_from_transform(msg)
            self.end_effector = np.matmul(self.base, base_to_end_effector)

        # if type(self.needle) is np.ndarray and type(self.end_effector) is np.ndarray:
        #     print('needle to end effector')
        #     print(np.matmul(np.linalg.inv(self.needle), self.end_effector))
        #     self.average[1:100] = self.average[0:99]
        #     self.average[0] = np.matmul(np.linalg.inv(self.needle), self.end_effector)
        #     print('average')
        #     print(np.average(self.average, axis=0))


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
            needle_grab = np.matmul(self.needle, self.needle_to_needle_grab_right)
            needle_to_needle_intermediate = np.array([[1, 0, 0, 0],
                                                     [ 0,  1, 0, 0],
                                                     [ 0, 0, 1, 0.1],
                                                     [ 0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00]])
            needle_intermediate = np.matmul(needle_to_needle_intermediate, needle_grab)
            path = [needle_intermediate, needle_grab]
            return path
        else:
            return None

    # path to the entry of the hole with the specified index
    def entry_path(self, index):
        if type(self.needle) is np.ndarray and type(self.end_effector) is np.ndarray:
            needle_tip = np.matmul(self.needle, self.needle_to_needle_tip)
            needle_tip_to_end_effector = np.matmul(np.linalg.inv(needle_tip), self.end_effector)

            entry_to_entry_intermediate = np.array([[1, 0, 0, -0.2],
                                                     [0, 1, 0, 0],
                                                     [0, 0, 1, 0],
                                                     [0, 0, 0, 1]])

            entry = np.matmul(self.world_to_entrys[index], np.array([[0, 0, 1, 0.0],
                                                                     [0, 1, 0, 0],
                                                                     [-1, 0, 0, 0],
                                                                     [0, 0, 0, 1]]))
#
            path = [np.matmul(np.matmul(entry, entry_to_entry_intermediate), needle_tip_to_end_effector), np.matmul(entry, needle_tip_to_end_effector)]
            return path
        else:
            return None

    # path for the arm to go through the hole and poke the needle to the other side
    def exit_path(self, index):
        if type(self.needle) is np.ndarray and type(self.end_effector) is np.ndarray:
            print('exit')
            needle_tip = np.matmul(self.needle, self.needle_to_needle_tip)
            needle_tip_to_end_effector = np.matmul(np.linalg.inv(needle_tip), self.end_effector)

            entry_to_entry_intermediate = np.array([[1, 0, 0, 0.05],
                                                     [0, 1, 0, 0],
                                                     [0, 0, 1, 0],
                                                     [0, 0, 0, 1]])

            entry = np.matmul(self.world_to_entrys[index], np.array([[0, 0, 1, 0.0],
                                                                     [0, 1, 0, 0],
                                                                     [-1, 0, 0, 0],
                                                                     [0, 0, 0, 1]]))

            exit_to_exit_intermediate = np.array([[1, 0, 0, -0.05],
                                                     [0, 1, 0, 0],
                                                     [0, 0, 1, 0],
                                                     [0, 0, 0, 1]])

            exit = np.matmul(self.world_to_exits[index], np.array([[0, 0, 1, 0.0],
                                                                     [0, 1, 0, 0],
                                                                     [1, 0, 0, 0],
                                                                     [0, 0, 0, 1]]))
#, np.matmul(exit, needle_tip_to_end_effector)
            path = [np.matmul(np.matmul(entry, entry_to_entry_intermediate), needle_tip_to_end_effector)]
            return path
        else:
            return None

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
    get_paths_node = GetPaths('psm2')
    rospy.spin()