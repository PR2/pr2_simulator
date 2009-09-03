#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

## Run the right arm through its full range of motion a few times.
## Limits were taken originally from pr2_defs/defs/arm_defs.xml, double check for changes.
## Works with l_arm_test_controller.xml controllers

PKG = 'pr2_arm_gazebo'
NAME = 'l_arm_setting'

import math
import roslib
import random
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')


import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from pr2_mechanism_controllers.msg import *

TEST_DURATION = 60.0
COMMAND_INTERVAL = 10.0
PI = 3.14159

CMD_SH_PAN_MIN     =  0*(PI/4-1.5)   #range [ PI/4-1.5   PI/4+1.5 ]
CMD_SH_LFT_MIN     =  -0.4       #range [ -0.4       1.5 ]
CMD_UA_ROL_MIN     =  1.55-2.35  #range [ 1.55-2.35  1.55+2.35 ]
CMD_EL_FLX_MIN     =  -2.3       #range [ -2.3       0.1 ]
CMD_FA_ROL_MIN     =  0*PI         #range [  ]
CMD_WR_FLX_MIN     =  -0.1       #range [ -0.1       2.2 ]
CMD_WR_ROL_MIN     =  0*PI         #range [  ]
CMD_GR_POS_MIN     =  0.0        #range [ 0          0.548 ]

CMD_SH_PAN_MAX     =  0*(PI/4+1.5)    #range [ PI/4-1.5   PI/4+1.5 ]
CMD_SH_LFT_MAX     =  1.5         #range [ -0.4       1.5 ]
CMD_UA_ROL_MAX     =  1.55+2.35   #range [ 1.55-2.35  1.55+2.35 ]
CMD_EL_FLX_MAX     =  0.1         #range [ -2.3       0.1 ]
CMD_FA_ROL_MAX     =  -0*PI         #range [  ]
CMD_WR_FLX_MAX     =  2.2         #range [ -0.1       2.2 ]
CMD_WR_ROL_MAX     =  -0*PI         #range [  ]
CMD_GR_POS_MAX     =  0.548       #range [ 0          0.548 ]

if __name__ == '__main__':
    pub_l_shoulder_pan   = rospy.Publisher("l_shoulder_pan_controller/set_command", Float64)
    pub_l_shoulder_lift  = rospy.Publisher("l_shoulder_lift_controller/set_command", Float64)
    pub_l_upper_arm_roll = rospy.Publisher("l_upper_arm_roll_controller/set_command", Float64)
    pub_l_elbow_flex     = rospy.Publisher("l_elbow_flex_controller/set_command", Float64)
    pub_l_elbow_roll     = rospy.Publisher("l_elbow_roll_controller/set_command", Float64)
    pub_l_wrist_flex     = rospy.Publisher("l_wrist_flex_controller/set_command", Float64)
    pub_l_wrist_roll     = rospy.Publisher("l_wrist_roll_controller/set_command", Float64)
    pub_l_gripper        = rospy.Publisher("l_gripper_controller/set_command", Float64)
    rospy.init_node(NAME, anonymous=True)
    timeout_t = time.time() + TEST_DURATION
    while time.time() < timeout_t:
        pub_l_shoulder_pan   .publish(Float64(CMD_SH_PAN_MIN))
        pub_l_shoulder_lift  .publish(Float64(CMD_SH_LFT_MIN))
        pub_l_upper_arm_roll .publish(Float64(CMD_UA_ROL_MIN))
        pub_l_elbow_flex     .publish(Float64(CMD_EL_FLX_MIN))
        pub_l_elbow_roll     .publish(Float64(CMD_FA_ROL_MIN))
        pub_l_wrist_flex     .publish(Float64(CMD_WR_FLX_MIN))
        pub_l_wrist_roll     .publish(Float64(CMD_WR_ROL_MIN))
        pub_l_gripper        .publish(Float64(CMD_GR_POS_MIN))
        time.sleep(COMMAND_INTERVAL)
        pub_l_shoulder_pan   .publish(Float64(CMD_SH_PAN_MAX))
        pub_l_shoulder_lift  .publish(Float64(CMD_SH_LFT_MAX))
        pub_l_upper_arm_roll .publish(Float64(CMD_UA_ROL_MAX))
        pub_l_elbow_flex     .publish(Float64(CMD_EL_FLX_MAX))
        pub_l_elbow_roll     .publish(Float64(CMD_FA_ROL_MAX))
        pub_l_wrist_flex     .publish(Float64(CMD_WR_FLX_MAX))
        pub_l_wrist_roll     .publish(Float64(CMD_WR_ROL_MAX))
        pub_l_gripper        .publish(Float64(CMD_GR_POS_MAX))
        time.sleep(COMMAND_INTERVAL)


