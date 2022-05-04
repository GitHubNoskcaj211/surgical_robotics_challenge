#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================
from surgical_robotics_challenge.kinematics.psmIK import *
from PyKDL import Frame, Rotation, Vector
import time
from surgical_robotics_challenge.utils.joint_pos_recorder import JointPosRecorder
from surgical_robotics_challenge.utils.joint_errors_model import JointErrorsModel
import numpy as np

jpRecorder = JointPosRecorder()

class PSMJointMapping:
    def __init__(self):
        self.idx_to_name = {0: 'baselink-yawlink',
                            1: 'yawlink-pitchbacklink',
                            2: 'pitchendlink-maininsertionlink',
                            3: 'maininsertionlink-toolrolllink',
                            4: 'toolrolllink-toolpitchlink',
                            5: 'toolpitchlink-toolyawlink'}

        self.name_to_idx = {'baselink-yawlink': 0,
                            'yawlink-pitchbacklink': 1,
                            'pitchendlink-maininsertionlink': 2,
                            'maininsertionlink-toolrolllink': 3,
                            'toolrolllink-toolpitchlink': 4,
                            'toolpitchlink-toolyawlink': 5}


pjm = PSMJointMapping()

def frame_to_mat(frame):
    mat = np.empty((4,4), dtype=np.float32)


def mat_to_frame(mat):
    frame = Frame()
    frame.p = Vector(mat[0,3],
                     mat[1,3],
                     mat[2,3])
    Quat = rot_mat_to_quat(mat[0:3,0:3])
    frame.M = Rotation.Quaternion(Quat[0],
                                  Quat[1],
                                  Quat[2],
                                  Quat[3])
    return frame

def rot_mat_to_quat(cp):
    R = Rotation(cp[0, 0], cp[0, 1], cp[0, 2],
                 cp[1, 0], cp[1, 1], cp[1, 2],
                 cp[2, 0], cp[2, 1], cp[2, 2])

    return R.GetQuaternion()

def np_mat_to_transform(cp):
    trans = Transform()
    trans.translation.x = cp[0, 3]
    trans.translation.y = cp[1, 3]
    trans.translation.z = cp[2, 3]

    Quat = rot_mat_to_quat(cp)

    trans.rotation.x = Quat[0]
    trans.rotation.y = Quat[1]
    trans.rotation.z = Quat[2]
    trans.rotation.w = Quat[3]
    return trans

def quaternion_to_rotation_matrix(Q):
    r = Rotation.from_quat([Q.x, Q.y, Q.z, Q.w])
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

class PSM:
    def __init__(self, client, name, add_joint_errors=False, save_jp=False):
        self.save_jp = save_jp
        self.client = client
        self.name = name
        self.base = self.client.get_obj_handle(name + '/baselink')
        self.target_IK = self.client.get_obj_handle(name + '_target_ik')
        self.palm_joint_IK = self.client.get_obj_handle(name + '_palm_joint_ik')
        self.target_FK = self.client.get_obj_handle(name + '_target_fk')
        self.sensor = self.client.get_obj_handle(name + '/Sensor0')
        self.actuators = []
        self.actuators.append(self.client.get_obj_handle(name + '/Actuator0'))
        time.sleep(0.5)
        self.grasped = [False, False, False]
        self.graspable_objs_prefix = ["Needle", "Thread", "Puzzle"]

        self.T_t_b_home = Frame(Rotation.RPY(3.14, 0.0, 1.57079), Vector(0.0, 0.0, -1.0))
        self.T_b_funk = Frame(Rotation.Quaternion(0.7071068, 0, 0, 0.7071068), Vector(0.0,0.0,0.0))

        # Transform of Base in World
        self._T_b_w = None
        # Transform of World in Base
        self._T_w_b = None
        self._base_pose_updated = False
        self._num_joints = 6
        self._ik_solution = np.zeros([self._num_joints])
        self._last_jp = np.zeros([self._num_joints])
        if add_joint_errors:
            self._errors_distribution_deg = [-5., -2. , 2., 5.]
        else:
            self._errors_distribution_deg = [0.] # No Error
        self._joints_error_mask = [1, 1, 0, 0, 0, 0] # Only apply error to joints with 1's
        self._joint_error_model = JointErrorsModel(self._num_joints, errors_distribution_deg=self._errors_distribution_deg)

    def set_home_pose(self, pose):
        self.T_t_b_home = pose

    def is_present(self):
        if self.base is None:
            return False
        else:
            return True

    def get_ik_solution(self):
        return self._ik_solution

    def get_T_b_w(self):
        self._update_base_pose()
        return self._T_b_w

    def get_T_w_b(self):
        self._update_base_pose()
        return self._T_w_b

    def _update_base_pose(self):
        if not self._base_pose_updated:
            p = self.base.get_pos()
            q = self.base.get_rot()
            P_b_w = Vector(p.x, p.y, p.z)
            R_b_w = Rotation.Quaternion(q.x, q.y, q.z, q.w)
            self._T_b_w = Frame(R_b_w, P_b_w)
            self._T_w_b = self._T_b_w.Inverse()
            self._base_pose_updated = True

    def run_grasp_logic(self, jaw_angle):
        for i in range(len(self.actuators)):
            if jaw_angle <= 0.2:
                if self.sensor is not None:
                    if self.sensor.is_triggered(i):
                        sensed_obj = self.sensor.get_sensed_object(i)
                        for s in self.graspable_objs_prefix:
                            if s in sensed_obj:
                                if not self.grasped[i]:
                                    qualified_name = sensed_obj
                                    self.actuators[i].actuate(qualified_name)
                                    self.grasped[i] = True
                                    print('Grasping Sensed Object Names', sensed_obj)
            else:
                if self.actuators[i] is not None:
                    self.actuators[i].deactuate()
                    if self.grasped[i] is True:
                        print('Releasing Grasped Object')
                    self.grasped[i] = False
                    # print('Releasing Actuator ', i)

    def servo_cp(self, T_req):
        T_t_f = self.get_T_w_b() * T_req

        if type(T_t_f) in [np.matrix, np.array]:
            T_t_f = convert_mat_to_frame(T_t_f)

        ik_solution = compute_IK(T_t_f)
        self._ik_solution = enforce_limits(ik_solution)
        self.servo_jp(self._ik_solution)

        ###  save jp

        if self.save_jp:
            jpRecorder.record(self._ik_solution)  ### record joint angles

    def servo_cv(self, twist):
        pass

    def optimize_jp(self, jp):
        # Optimizing the tool shaft roll angle
        pass

    def servo_jp(self, jp):
        jp = self._joint_error_model.add_to_joints(jp, self._joints_error_mask)
        self.base.set_joint_pos(0, jp[0])
        self.base.set_joint_pos(1, jp[1])
        self.base.set_joint_pos(2, jp[2])
        self.base.set_joint_pos(3, jp[3])
        self.base.set_joint_pos(4, jp[4])
        self.base.set_joint_pos(5, jp[5])

    def servo_jv(self, jv):
        print("Setting Joint Vel", jv)
        self.base.set_joint_vel(0, jv[0])
        self.base.set_joint_vel(1, jv[1])
        self.base.set_joint_vel(2, jv[2])
        self.base.set_joint_vel(3, jv[3])
        self.base.set_joint_vel(4, jv[4])
        self.base.set_joint_vel(5, jv[5])

    def set_jaw_angle(self, jaw_angle):
        self.base.set_joint_pos('toolyawlink-toolgripper1link', jaw_angle)
        self.base.set_joint_pos('toolyawlink-toolgripper2link', jaw_angle)
        self.run_grasp_logic(jaw_angle)

    def measured_cp(self):
        jp = self.measured_jp()
        jp.append(0.0)
        return compute_FK(jp, 7)

    def measured_jp(self):
        j0 = self.base.get_joint_pos(0)
        j1 = self.base.get_joint_pos(1)
        j2 = self.base.get_joint_pos(2)
        j3 = self.base.get_joint_pos(3)
        j4 = self.base.get_joint_pos(4)
        j5 = self.base.get_joint_pos(5)
        q = [j0, j1, j2, j3, j4, j5]
        q = self._joint_error_model.remove_from_joints(q, self._joints_error_mask)
        return q

    def measured_jv(self):
        j0 = self.base.get_joint_vel(0)
        j1 = self.base.get_joint_vel(1)
        j2 = self.base.get_joint_vel(2)
        j3 = self.base.get_joint_vel(3)
        j4 = self.base.get_joint_vel(4)
        j5 = self.base.get_joint_vel(5)
        return [j0, j1, j2, j3, j4, j5]

    def get_joint_names(self):
        return self.base.get_joint_names()
