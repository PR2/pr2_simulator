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

## Gazebo collision validation 
PKG = 'pr2_gazebo'
NAME = 'test_slide'

import roslib
roslib.load_manifest(PKG)

import unittest, sys, os, math
import time
import rospy, rostest
from nav_msgs.msg import Odometry

TEST_DURATION = 90.0
TARGET_X = -6.0  #contains offset specified in P3D for base, alternatively, use the gripper roll ground truths
TARGET_Y = 0.0  #contains offset specified in P3D for base, alternatively, use the gripper roll ground truths
TARGET_Z = 3.8
TARGET_RAD = 4.5
CUP_HEIGHT = 4.0
MIN_HITS = 200.0
MIN_RUNS = 20.0

class TestSlide(unittest.TestCase):
    def __init__(self, *args):
        super(TestSlide, self).__init__(*args)
        self.success = False
        self.fail = False
        self.hits = 0
        self.runs = 0
        self.print_header = False
        
    def positionInput(self, p3d):
        if not self.print_header: 
          self.print_header = True
          rospy.loginfo("runs   hits   x   y   z    dx    dy    dz    dr   r_tol")

        self.runs = self.runs + 1
        #if (pos.frame == 1):
        dx = p3d.pose.pose.position.x - TARGET_X
        dy = p3d.pose.pose.position.y - TARGET_Y
        dz = p3d.pose.pose.position.z - TARGET_Z
        d = math.sqrt((dx * dx) + (dy * dy)) #+ (dz * dz))

        rospy.loginfo("self.runs, self.hits, "\
              "p3d.pose.pose.position.x , p3d.pose.pose.position.y , p3d.pose.pose.position.z, "\
              "dx , dy , dz , d, TARGET_RAD")

        if (d < TARGET_RAD and abs(dz) < CUP_HEIGHT):
            self.hits = self.hits + 1
            if (self.runs > 10 and self.runs < 50):
                rospy.logwarn("Got to goal too quickly! (",self.runs,")")
                self.success = False
                self.fail = True

            if (self.hits > MIN_HITS):
                if (self.runs > MIN_RUNS):
                    self.success = True

        
    
    def test_slide(self):
        print "LINK\n"
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.positionInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_DURATION
        while not rospy.is_shutdown() and not self.success and not self.fail and time.time() < timeout_t:
            time.sleep(0.1)
        time.sleep(2.0)
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestSlide, sys.argv) #, text_mode=True)


