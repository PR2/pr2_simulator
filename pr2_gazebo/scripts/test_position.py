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

## Gazebo tug arms for navigation

PKG = 'pr2_gazebo'
NAME = 'test_position'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


if __name__ == '__main__':
    pub_r_shoulder_pan   = rospy.Publisher("r_shoulder_pan_position_controller/command", Float64)
    pub_r_shoulder_lift  = rospy.Publisher("r_shoulder_lift_position_controller/command", Float64)
    pub_r_upper_arm_roll = rospy.Publisher("r_upper_arm_roll_position_controller/command", Float64)
    pub_r_elbow_flex     = rospy.Publisher("r_elbow_flex_position_controller/command", Float64)
    pub_r_forearm_roll   = rospy.Publisher("r_forearm_roll_position_controller/command", Float64)
    pub_r_wrist_flex     = rospy.Publisher("r_wrist_flex_position_controller/command", Float64)
    pub_r_wrist_roll     = rospy.Publisher("r_wrist_roll_position_controller/command", Float64)
    pub_r_gripper        = rospy.Publisher("r_gripper_position_controller/command", Float64)

    #rospy.Subscriber("r_gripper_palm_pose_ground_truth", Odometry, p3dReceived)
    rospy.init_node(NAME, anonymous=True)

    time.sleep(3) #wait for node init
    start_time = rospy.get_rostime().to_seconds()

    print "start_time: ", start_time, " " ,rospy.get_rostime()

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0.2))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(-0.3))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(-0.3))
    pub_r_shoulder_lift  .publish(Float64(0.3))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(-0.3))
    pub_r_shoulder_lift  .publish(Float64(-0.2))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0.3))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(-0.3))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(-0.3))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0.3))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(-0.3))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(-0.3))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

    print "current_time: ", rospy.get_rostime().to_seconds()
    timeout = timeout + 5;
    pub_r_shoulder_pan   .publish(Float64(0))
    pub_r_shoulder_lift  .publish(Float64(0))
    pub_r_upper_arm_roll .publish(Float64(0))
    pub_r_elbow_flex     .publish(Float64(0))
    pub_r_forearm_roll   .publish(Float64(0))
    pub_r_wrist_flex     .publish(Float64(0))
    pub_r_wrist_roll     .publish(Float64(0))
    pub_r_gripper        .publish(Float64(0))
    while (rospy.get_rostime().to_seconds() - start_time) < timeout:
      time.sleep(0.01);

