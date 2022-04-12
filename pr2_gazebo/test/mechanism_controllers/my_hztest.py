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
NAME = 'my_hztest'

import math
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from tf.msg import tfMessage
from pr2_mechanism_msgs.msg import MechanismState

MIN_MSGS        = 100
TEST_TIMEOUT    = 100000000000.0

class MyHzTest(unittest.TestCase):
    def __init__(self, *args):
        super(MyHzTest, self).__init__(*args)
        self.success = False
        self.count = 0

        #for time tracking
        self.started = False
        self.start_time = 0
        self.end_time = 0

    def Input(self, msg):
        self.count += 1;

        if not self.started:
          self.start_time = rospy.get_rostime().to_sec()
          print(" got first message at: ",self.start_time, " sec")
          self.started = True

        if self.count >= MIN_MSGS:
          self.end_time = rospy.get_rostime().to_sec()
          print(" passed minimum ",self.count," messages at ",self.count / (self.end_time - self.start_time), " Hz")
          self.success = True
        #else:
          #print " got ",self.count," messages at ",self.count / (rospy.get_rostime().to_sec() - self.start_time), " Hz"

    def test_hz(self):
        print("LNK\n")
        #rospy.Subscriber("/tf", tfMessage, self.Input)
        rospy.Subscriber("/mechanism_state", MechanismState, self.Input)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_TIMEOUT
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(1.0)

        self.assert_(self.success)

if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], MyHzTest, sys.argv) #, text_mode=True)

