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
        self.needle = None
        rospy.Subscriber('/CRTK/Needle/measured_cp', TransformStamped, self.set_needle_position, queue_size=1)

        self.base = None
        rospy.Subscriber('/CRTK/' + arm_frame_id + '/T_b_t_w', TransformStamped, self.set_base_position, queue_size=1)

        self.needle_to_needle_grab_right = np.array([[0,  1,  0,  -0.1],
                                                     [ 1,  0, 0, 0.01],
                                                     [0, 0, -1, -.03],
                                                     [ 0,0,0,1]])

        self.needle_to_needle_grab_left = np.array([[.874,  -.485,  0,  0.0485],
                                                     [ -.485,  -.874, 0, 0.0874],
                                                     [0, 0, -1, -.03],
                                                     [ 0,0,0,1]])

        self.needle_to_needle_tip = np.array([[0.866,0,-0.5,0.05],[-0.5,0,-0.866,.0866],[0,1,0,0],[0,0,0,1]])

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
            needle_grab_to_needle_intermediate = np.array([[1, 0, 0, 0],
                                                     [ 0,  1, 0, 0],
                                                     [ 0, 0, 1, -0.1],
                                                     [ 0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00]])
            needle_intermediate = np.matmul(needle_grab, needle_grab_to_needle_intermediate)
            path = [needle_intermediate, needle_grab]
            return path
        else:
            return None

    # path to grab the needle with the left arm
    def grab_needle_left(self):
        if type(self.needle) is np.ndarray:
            needle_grab = np.matmul(self.needle, self.needle_to_needle_grab_left)
            needle_grab_to_needle_intermediate = np.array([[1, 0, 0, 0],
                                                     [ 0,  1, 0, 0],
                                                     [ 0, 0, 1, -0.1],
                                                     [ 0.0000000e+00,  0.0000000e+00,  0.0000000e+00,  1.0000000e+00]])
            needle_intermediate = np.matmul(needle_grab, needle_grab_to_needle_intermediate)
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
            path = []
            start_angle = math.pi / 3.0
            end_angle = -math.pi / 6.0
            diff = self.world_to_exits[index][0:3,3] - self.world_to_entrys[index][0:3,3]
            d = math.sqrt(np.dot(diff,diff))

            entry = np.matmul(self.world_to_entrys[index], np.array([[0, 0, 1, 0.0],
                                                                     [0, 1, 0, 0],
                                                                     [-1, 0, 0, 0],
                                                                     [0, 0, 0, 1]]))

            exit = np.matmul(self.world_to_exits[index], np.array([[0, 0, -1, 0.0],
                                                                     [0, 1, 0, 0],
                                                                     [1, 0, 0, 0],
                                                                     [0, 0, 0, 1]]))

            phantom_middle = np.matmul(np.matmul(entry,
                                                                     np.array([[.707, 0, -.707, 0.0],
                                                                     [0, 1, 0, 0],
                                                                     [.707, 0, .707, 0],
                                                                     [0, 0, 0, 1]])),
                                                                     np.array([[1, 0, 0, d / 2.0],
                                                                    [0, 1, 0, 0],
                                                                    [0, 0, 1, 0],
                                                                    [0, 0, 0, 1]]))
            for angle in np.linspace(start_angle, end_angle, 10):
                needle_intermediate = np.matmul(self.needle, np.array([[math.cos(angle),0,-math.sin(angle),math.sin(angle)/10.0],[-math.sin(angle),0,-math.cos(angle),math.cos(angle) / 10.0],[0,1,0,0],[0,0,0,1]]))
                needle_intermediate_to_end_effector = np.matmul(np.linalg.inv(needle_intermediate), self.end_effector)



                path.append(np.matmul(phantom_middle, needle_intermediate_to_end_effector))
            return path
        else:
            return None

    # path for the arm to go through the hole and poke the needle to the other side
    def exit_path_new(self, index):
        if type(self.needle) is np.ndarray and type(self.end_effector) is np.ndarray:
            path = []
            start_angle = math.pi / 6.0
            end_angle = -math.pi / 3.0
            for angle in np.linspace(start_angle, end_angle, 24):
                needle_intermediate1 = np.matmul(self.needle, np.array([[math.cos(angle),0,-math.sin(angle),math.sin(angle)/10.0],[-math.sin(angle),0,-math.cos(angle),math.cos(angle) / 10.0],[0,1,0,0],[0,0,0,1]]))
                needle_intermediate1_to_end_effector = np.matmul(np.linalg.inv(needle_intermediate1), self.end_effector)
                angle = math.pi / 2.0 + angle
                needle_intermediate2 = np.matmul(self.needle, np.array([[math.cos(angle),0,-math.sin(angle),math.sin(angle)/10.0],[-math.sin(angle),0,-math.cos(angle),math.cos(angle) / 10.0],[0,1,0,0],[0,0,0,1]]))
                needle_intermediate2_to_end_effector = np.matmul(np.linalg.inv(needle_intermediate2), self.end_effector)

                entry = np.matmul(self.world_to_entrys[index], np.array([[0, 0, 1, 0.0],
                                                                         [0, 1, 0, 0],
                                                                         [-1, 0, 0, 0],
                                                                         [0, 0, 0, 1]]))

                exit = np.matmul(self.world_to_exits[index], np.array([[0, 0, -1, 0.0],
                                                                         [0, 1, 0, 0],
                                                                         [1, 0, 0, 0],
                                                                         [0, 0, 0, 1]]))

                spot1 = np.matmul(entry, needle_intermediate1_to_end_effector)
                spot2 = np.matmul(exit, needle_intermediate2_to_end_effector)
                path.append(spot1 * 0.5 + spot2 * 0.5)
            return path
        else:
            return None

    # return the arm to home (above the hole for the index)
    def arm_home_path(self, index):
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
