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

## Gazebo test base controller vw

PKG = 'pr2_gazebo'
NAME = 'test_base_odomx_gt'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, time
import rospy, rostest

from geometry_msgs.msg import Twist,Vector3
from nav_msgs.msg import Odometry

TEST_DURATION   = 10.0

TARGET_VX       =  0.5
TARGET_VY       =  0.0
TARGET_VW       =  0.0
TARGET_DURATION = 2.0
TARGET_TOL      = 0.2

from test_base import BaseTest, E, Q
class X_GT(BaseTest):
    def __init__(self, *args):
        super(X_GT, self).__init__(*args)

    def test_base(self):
        self.init_ros(NAME)
        timeout_t = time.time() + TEST_DURATION
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            #do not start commanding base until p3d and odom are initialized
            if self.p3d_initialized == True and self.odom_initialized == True:
              self.pub.publish(Twist(Vector3(TARGET_VX,TARGET_VY, 0), Vector3(0,0,TARGET_VW)))
            time.sleep(0.1)
            #self.debug_pos()
            # display what the odom error is
            print " error   " + " x: " + str(self.odom_x - self.p3d_x) + " y: " + str(self.odom_y - self.p3d_y) + " t: " + str(self.odom_t - self.p3d_t)

        # check total error
        total_error = abs(self.odom_x - self.p3d_x) + abs(self.odom_y - self.p3d_y) + abs(self.odom_t - self.p3d_t)
        if total_error < TARGET_TOL:
            self.success = True

        self.assert_(self.success)
        
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], X_GT, sys.argv) #, text_mode=True)


