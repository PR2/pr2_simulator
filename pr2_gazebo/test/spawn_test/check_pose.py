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

## Gazebo test arm controller
##   sends posision
##   checks to see if P3D returns corresponding ground truth within TARGET_TOL of TARGET_VW
##          for a duration of TARGET_DURATION seconds

PKG = 'pr2_gazebo'
NAME = 'check_pose'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import *
from numpy import *


GRP_CMD_POS     = 0.03

TARGET_DURATION = 1.0
ROT_TARGET_TOL      = 0.01  #empirical test result john - 20090110
POS_TARGET_TOL      = 0.01  #empirical test result john - 20090110
TEST_TIMEOUT    = 100.0

# pre-recorded poses for above commands
TARGET_BASE_TX     = 2.0
TARGET_BASE_TY     = 2.0
TARGET_BASE_TZ     = 0.0
TARGET_BASE_QX     = 0.0
TARGET_BASE_QY     = 0.0
TARGET_BASE_QZ     = 0.0
TARGET_BASE_QW     = 1.0

class PoseTest(unittest.TestCase):
    def __init__(self, *args):
        super(PoseTest, self).__init__(*args)
        self.base_success = False
        self.reached_target_base = False
        self.duration_start_base = 0

    def printP3D(self, p3d):
        print "pose ground truth received"
        print "P3D pose translan: " + "x: " + str(p3d.pose.pose.position.x)
        print "                   " + "y: " + str(p3d.pos.position.y)
        print "                   " + "z: " + str(p3d.pose.pose.position.z)
        print "P3D pose rotation: " + "x: " + str(p3d.pose.pose.orientation.x)
        print "                   " + "y: " + str(p3d.pose.pose.orientation.y)
        print "                   " + "z: " + str(p3d.pose.pose.orientation.z)
        print "                   " + "w: " + str(p3d.pose.pose.orientation.w)
        print "P3D rate translan: " + "x: " + str(p3d.vel.vel.vx)
        print "                   " + "y: " + str(p3d.vel.vel.vy)
        print "                   " + "z: " + str(p3d.vel.vel.vz)
        print "P3D rate rotation: " + "x: " + str(p3d.vel.ang_vel.vx)
        print "                   " + "y: " + str(p3d.vel.ang_vel.vy)
        print "                   " + "z: " + str(p3d.vel.ang_vel.vz)

    def baseP3dInput(self, p3d):
        i = 0
        pos_error = abs(p3d.pose.pose.position.x - TARGET_BASE_TX) + \
                    abs(p3d.pose.pose.position.y - TARGET_BASE_TY) + \
                    abs(p3d.pose.pose.position.z - TARGET_BASE_TZ) * 0 # ignore BASE_Z

        #target pose rotation matrix
        target_q = [TARGET_BASE_QX  \
                   ,TARGET_BASE_QY  \
                   ,TARGET_BASE_QZ  \
                   ,TARGET_BASE_QW]

        #p3d pose quaternion
        p3d_q     = [p3d.pose.pose.orientation.x  \
                    ,p3d.pose.pose.orientation.y  \
                    ,p3d.pose.pose.orientation.z  \
                    ,p3d.pose.pose.orientation.w]

        # get error euler angles by inverting the target rotation matrix and multiplying by p3d quaternion
        target_q_inv = quaternion_inverse(target_q)
        rot_euler = euler_from_quaternion( quaternion_multiply(p3d_q, target_q_inv) )
        rot_error = abs( rot_euler[0] ) + \
                    abs( rot_euler[1] ) + \
                    abs( rot_euler[2] )

        print " base Error pos: " + str(pos_error) + " rot: " + str(rot_error)

        #self.printP3D(p3d) #for getting new valid data

        # has to reach target vw and maintain target vw for a duration of TARGET_DURATION seconds
        if self.reached_target_base:
          print " base duration: " + str(time.time() - self.duration_start_base)
          if rot_error < ROT_TARGET_TOL and pos_error < POS_TARGET_TOL:
            if time.time() - self.duration_start_base > TARGET_DURATION:
              self.base_success = True
          else:
            # failed to maintain target vw, reset duration
            self.base_success = False
            self.reached_target_base = False
        else:
          if rot_error < ROT_TARGET_TOL and pos_error < POS_TARGET_TOL:
            print 'success base'
            self.reached_target_base = True
            self.duration_start_base = time.time()
    
    def test_arm(self):
        print "LNK\n"
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.baseP3dInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_TIMEOUT
        while not rospy.is_shutdown() and not self.base_success and time.time() < timeout_t:
            time.sleep(1.0)
        self.assert_(self.base_success)
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], PoseTest, sys.argv) #, text_mode=True)
