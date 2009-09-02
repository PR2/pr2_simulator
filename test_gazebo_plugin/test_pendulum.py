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

## Gazebo test Pendulum physics
##          see error in joint constraint for ODE

PKG = 'test_gazebo_plugin'
NAME = 'test_pendulum'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from nav_msgs.msg import *

TOLERANCE = 0.01
MAX_ERROR = 0.02
TEST_DURATION = 20.0

class E:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class Q:
    def __init__(self,x,y,z,u):
        self.x = x
        self.y = y
        self.z = z
        self.u = u
    def normalize(self):
        s = math.sqrt(self.u * self.u + self.x * self.x + self.y * self.y + self.z * self.z)
        self.u /= s
        self.x /= s
        self.y /= s
        self.z /= s
    def getEuler(self):
        self.normalize()
        squ = self.u
        sqx = self.x
        sqy = self.y
        sqz = self.z
        # init euler angles
        vec = E(0,0,0)
        # Roll
        vec.x = math.atan2(2 * (self.y*self.z + self.u*self.x), squ - sqx - sqy + sqz);
        # Pitch
        vec.y = math.asin(-2 * (self.x*self.z - self.u*self.y));
        # Yaw
        vec.z = math.atan2(2 * (self.x*self.y + self.u*self.z), squ + sqx - sqy - sqz);
        return vec


class PendulumTest(unittest.TestCase):
    def __init__(self, *args):
        super(PendulumTest, self).__init__(*args)
        self.success = False
        self.reached_target_vw = False
        self.duration_start = 0
        self.error1_total = 0
        self.error1_count = 0
        self.error2_total = 0
        self.error2_count = 0
        self.error1_max = 0
        self.error2_max = 0
        self.min_runs     = 1000  # average error over this many runs



    #def printPendulum(self, p3d):
        #print "P3D pose translan: " + "x: " + str(p3d.pos.position.x)
        #print "                   " + "y: " + str(p3d.pos.position.y)
        #print "                   " + "z: " + str(p3d.pos.position.z)
        #print "P3D pose rotation: " + "x: " + str(p3d.pos.orientation.x)
        #print "                   " + "y: " + str(p3d.pos.orientation.y)
        #print "                   " + "z: " + str(p3d.pos.orientation.z)
        #print "                   " + "w: " + str(p3d.pos.orientation.w)
        #print "P3D rate translan: " + "x: " + str(p3d.vel.vel.x)
        #print "                   " + "y: " + str(p3d.vel.vel.y)
        #print "                   " + "z: " + str(p3d.vel.vel.z)
        #print "P3D rate rotation: " + "x: " + str(p3d.vel.ang_vel.vx)
        #print "                   " + "y: " + str(p3d.vel.ang_vel.vy)
        #print "                   " + "z: " + str(p3d.vel.ang_vel.vz)


    def p3dInput1(self, p3d):
        #print "link1 pose ground truth received"
        #self.printPendulum(p3d)
        tmpx = p3d.pose.pose.position.x
        tmpy = p3d.pose.pose.position.y
        tmpz = p3d.pose.pose.position.z - 2.0

        self.error1_total += math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz)
        self.error1_count += 1
        if math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz) > self.error1_max:
            self.error1_max =  math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz)
 
    def p3dInput2(self, p3d):
        tmpx = p3d.pose.pose.position.x
        tmpy = p3d.pose.pose.position.y
        tmpz = p3d.pose.pose.position.z - 2.0

        self.error2_total += math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz)
        self.error2_count += 1
        if math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz) > self.error2_max:
            self.error2_max =  math.sqrt(tmpx*tmpx+tmpy*tmpy+tmpz*tmpz)

    def test_pendulum(self):
        print "LNK\n"
        rospy.Subscriber("/link1_pose", Odometry, self.p3dInput1)
        rospy.Subscriber("/link2_pose", Odometry, self.p3dInput2)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_DURATION
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            if self.error1_count > 0 and self.error2_count > 0:
              print  "E1 count:" + str(self.error1_count) \
                                 + " error:" + str(self.error1_total) \
                                 + " avg err:" + str(self.error1_total / self.error1_count) \
                                 + " max err:" + str(self.error1_max) \
                  + " E2 count:" + str(self.error2_count) \
                                 + " error:" + str(self.error2_total) \
                                 + " avg err:" + str(self.error2_total / self.error2_count) \
                                 + " max err:" + str(self.error2_max)
              if self.error2_count > self.min_runs and self.error1_count > self.min_runs:
                if self.error1_total / self.error1_count < TOLERANCE and self.error2_total / self.error2_count < TOLERANCE:
                  if self.error1_max < MAX_ERROR and self.error2_max < MAX_ERROR:
                    self.success = True
            time.sleep(0.1)
        self.assert_(self.success)
        
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], PendulumTest, sys.argv) #, text_mode=True)


