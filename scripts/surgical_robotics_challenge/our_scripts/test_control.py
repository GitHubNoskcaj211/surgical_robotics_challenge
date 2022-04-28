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

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import rospy
import math
import numpy as np
from PyKDL import Rotation

rospy.init_node("sur_chal_crtk_test")

namespace = "/CRTK/"
arm_name = "psm2"
# measured_cp_name = namespace + arm_name + "/measured_cp"
measured_cp_name = namespace + arm_name + "/measured_cp"
servo_cp_name = namespace + arm_name + "/servo_cp"


def measured_cp_cb(msg):
    # print('actual')
    # print(msg)
    pass

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

measured_cp_sub = rospy.Subscriber(
    measured_cp_name, TransformStamped, measured_cp_cb, queue_size=1)

servo_cp_pub = rospy.Publisher(servo_cp_name, TransformStamped, queue_size=1)

rate = rospy.Rate(50)

servo_cp_msg = np_mat_to_transform(np.array([[-0.36093866,  0.93243638, -0.01687898,  0.64643521],
       [ 0.92111654,  0.35360951, -0.16280056, -0.570517  ],
       [-0.1458328 , -0.07430941, -0.98651398, -2.73269666],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]))
# servo_cp_msg = TransformStamped()
# R_7_0 = Rotation.RPY(0.0, 0.0, 0.0)
# servo_cp_msg.transform.rotation.x = -0.51
# servo_cp_msg.transform.rotation.y = 0.82
# servo_cp_msg.transform.rotation.z = -.23
# servo_cp_msg.transform.rotation.w = -0.07
# servo_cp_msg.transform.translation.x = -0.148
# servo_cp_msg.transform.translation.y = -0.23
# servo_cp_msg.transform.translation.z = -0.07

# NO CONVERSION (IN BASELINK)
# servo_cp_msg.transform.translation.x = 0.2
# servo_cp_msg.transform.translation.y = 0.2
# servo_cp_msg.transform.translation.z = -1.0
# R_7_0 = Rotation.RPY(1.56, -0.0, 3.14)
# servo_cp_msg.transform.rotation.x = R_7_0.GetQuaternion()[0]
# servo_cp_msg.transform.rotation.y = R_7_0.GetQuaternion()[1]
# servo_cp_msg.transform.rotation.z = R_7_0.GetQuaternion()[2]
# servo_cp_msg.transform.rotation.w = R_7_0.GetQuaternion()[3]

# SAME SPOT IN WORLD
# servo_cp_msg.transform.translation.x = -.5
# servo_cp_msg.transform.translation.y = -.7
# servo_cp_msg.transform.translation.z = -2.56
# servo_cp_msg.transform.rotation.x = 0.3969157
# servo_cp_msg.transform.rotation.y = 0.1199314
# servo_cp_msg.transform.rotation.z = -0.2335806
# servo_cp_msg.transform.rotation.w = 0.8794967



# servo_cp_msg.transform.translation.x = -0.00317593690935
# servo_cp_msg.transform.translation.y = 0.0779744123351
# servo_cp_msg.transform.translation.z = -0.992181833552
# servo_cp_msg.transform.rotation.x = 0.711582240422
# servo_cp_msg.transform.rotation.y = 0.687330061807
# servo_cp_msg.transform.rotation.z = -0.0515409510565
# servo_cp_msg.transform.rotation.w = -0.136277773744

# servo_cp_msg.transform.translation.x = .9
# servo_cp_msg.transform.translation.y = 1.5
# servo_cp_msg.transform.translation.z = -0.671019715584
# servo_cp_msg.transform.rotation.x = 0.745155668887
# servo_cp_msg.transform.rotation.y = 0.622522005054
# servo_cp_msg.transform.rotation.z = -0.150162719146
# servo_cp_msg.transform.rotation.w = -0.186173414128

# servo_cp_msg.transform.translation.x = 0.1
# servo_cp_msg.transform.translation.y = 0.1
# servo_cp_msg.transform.translation.z = -1.0
# servo_cp_msg.transform.rotation.x = 0.0
# servo_cp_msg.transform.rotation.y = 0.0
# servo_cp_msg.transform.rotation.z = 0.0
# servo_cp_msg.transform.rotation.w = 1.0

while not rospy.is_shutdown():
    servo_cp_pub.publish(servo_cp_msg)
    print('requested')
    print(servo_cp_msg)
    rate.sleep()
