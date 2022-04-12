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

## Gazebo test cameras validation 

PKG = 'pr2_gazebo'
NAME = 'test_scan'

import math
import roslib
roslib.load_manifest(PKG)


import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from sensor_msgs.msg import LaserScan

TEST_DURATION  = 25
ERROR_TOL      = 0.05
FAIL_COUNT_TOL = 10

inf = float('inf')
TARGET_RANGES = [
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf,
2.04421663284, 2.03210663795, 2.0194671154, 2.0155646801, 1.99427628517, 2.00035619736, 1.98695075512, 
1.97852289677, 1.96805429459, 1.9588958025, 1.95648872852, 1.94692718983, 1.93678343296, 1.91842794418, 
1.92827701569, 1.91704964638, 1.91358006001, 1.92673146725, 1.94037222862, 1.9256892204284668, inf, inf,
inf, inf, inf, 2.14445352554, 2.08714962006, 2.05095148087, 2.02395749092, 2.00428080559,
1.97976839542, 1.95980763435, 1.9413497448, 1.93820095062, 1.91665804386, 1.92057430744, 1.9003251791, 
1.88783442974, 1.8934186697, 1.8741286993, 1.86705768108, 1.85493171215, 1.85695803165, 1.85333991051, 
1.83796668053, 1.84245407581, 1.84312713146, 1.84570860863, 1.82822322845, 1.83273553848, 1.83729255199, 
1.83289527893, 1.8334633112, 1.83232152462, 1.82959234715, 1.8262963295, 1.83373129368, 1.83350098133, 
1.83656823635, 1.83848130703, 1.83709740639, 1.85071253777, 1.84394586086, 1.86468243599, 1.85489320755, 
1.85955429077, 1.86517083645, 1.88544261456, 1.89780557156, 1.89888262749, 1.92238008976, 1.922524333, 
1.93953430653, 1.948564291, 1.96180999279, 1.98394215107, 1.99410378933, 2.02381777763, 2.05232095718, 
2.09428310394, 2.14106678963, 2.51629209518, 2.51145839691, 2.50856781006, 2.49926424026, 2.51138210297, 
2.50430202484, 2.50846076012, 2.52427816391, 2.52740097046, 2.5371067524, 2.54831123352, 2.56848597527, 
2.59808444977, 2.62111496925, 2.68968343735, inf, inf, inf, inf,
inf, inf, inf, inf, inf, 2.72182154655, 2.7262597084,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, 1.80504393578, 1.76711213589, 1.73867917061, 1.7188462019, 1.706569314,
1.69170439243, 1.67706441879, 1.66418457031, 1.6565977335, 1.64732301235, 1.64360272884, 1.63184809685, 
1.63481676579, 1.62855827808, 1.62778234482, 1.61982572079, 1.6105659008, 1.6124740839, 1.60954415798, 
1.6100025177, 1.60651493073, 1.59700119495, 1.60250425339, 1.60432767868, 1.602414608, 1.60677278042, 
1.61422848701, 1.61278867722, 1.61149096489, 1.61712217331, 1.61782002449, 1.63084983826, 1.63016283512, 
1.64222669601, 1.64433014393, 1.65826809406, 1.66858124733, 1.67725527287, 1.68793737888, 1.70181775093, 
1.71679854393, 1.74195897579, 1.76237237453, 1.80084371567, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf]






TARGET_INTENSITIES = [
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 250.002059937, 500.003509521,
500.005279541, 499.994689941, 500.009216309, 499.99597168, 499.988525391, 499.999633789, 499.992553711, 
499.995391846, 499.997558594, 500.000244141, 500.001342773, 499.990570068, 499.999420166, 499.998443604, 
500.003082275, 500.000579834, 500.011169434, 499.998931885, 249.997711182, 0.0, 0.0,
0.0, 1500.00146484, 3000.0, 3000.00292969, 3000.00268555, 3000.00878906, 3000.00415039,
3000.00097656, 2999.99853516, 3000.00805664, 3000.00146484, 3000.00415039, 2999.99633789, 2999.99609375, 
3000.00561523, 2999.99438477, 2999.99584961, 2999.99951172, 2999.99536133, 3000.00341797, 3000.00146484, 
2999.99536133, 2999.99633789, 3000.00048828, 3000.00268555, 2999.99243164, 3000.00415039, 2999.99853516, 
3000.00439453, 3000.00708008, 2999.99316406, 3000.00366211, 3000.0, 3000.0065918, 2999.99780273, 
3000.0, 2999.99121094, 2999.99902344, 3000.00195312, 2999.98901367, 2999.9934082, 2999.99951172, 
2999.99584961, 3000.00366211, 2999.99829102, 2999.99902344, 3000.00292969, 2999.99853516, 2999.99902344, 
2999.99658203, 2999.99829102, 2999.99658203, 3000.00146484, 3000.00195312, 3000.00878906, 2999.99414062, 
3000.00097656, 2499.99438477, 2000.00012207, 2000.00183105, 1999.98620605, 1999.99951172, 1999.99206543, 
2000.00280762, 1999.99804688, 2000.00195312, 1999.99401855, 1999.99853516, 1999.9901123, 1999.99731445, 
2000.00256348, 2000.00927734, 1000.00360107, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 1000.0012207, 1999.9967041, 1000.00085449,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 500.003265381, 999.998596191, 999.997680664, 999.991210938, 999.996032715, 1000.00134277,
1000.00958252, 999.99786377, 1000.0055542, 999.996276855, 1000.00384521, 999.996765137, 1000.00292969, 
1000.00128174, 999.996154785, 1000.0032959, 1000.00061035, 999.99609375, 999.99621582, 999.996520996, 
1000.00500488, 999.997253418, 999.983886719, 1000.00042725, 999.998596191, 999.996887207, 999.992370605, 
1000.00213623, 1000.00799561, 999.997680664, 1000.00170898, 1000.00073242, 999.996398926, 1000.00830078, 
1000.00238037, 999.993774414, 999.993713379, 999.989990234, 999.996765137, 1000.00146484, 1000.00634766, 
999.990966797, 1000.00305176, 999.996704102, 499.996307373, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ]

# indigo gazebo(2) uses 10.0 for inf data
if os.environ['ROS_DISTRO'] <= 'indigo':
    TARGET_RANGES = map(lambda x: x if x != inf else 10.0, TARGET_RANGES)

class PointCloudTest(unittest.TestCase):
    def __init__(self, *args):
        super(PointCloudTest, self).__init__(*args)
        self.success = False


    def printPointCloud(self, cloud):
        print("[")
        i = 0
        for pt in cloud.ranges:
            sys.stdout.write(str(pt) + ", ")
            i = i + 1
            if ((i % 7) == 0):
                print("") #newline
        print("]")

        print("[")
        i = 0
        for pt in cloud.intensities:
            sys.stdout.write(str(pt) + ", ")
            i = i + 1
            if ((i % 7) == 0):
                print("") #newline
        print("]")


    def pointInput(self, cloud):
        i = 0
        range_fail_count = 0
        print("Input laser scan received")
        self.printPointCloud(cloud)  #uncomment to capture new data
        while (i < len(cloud.ranges) and i < len(TARGET_RANGES)):
            d = cloud.ranges[i] - TARGET_RANGES[i]
            if ((d < - ERROR_TOL) or (d > ERROR_TOL)):
                range_fail_count += 1
                print("range_fail_count:" + str(range_fail_count) + " failed. error:" + str(d) + " exceeded tolerance:" + str(ERROR_TOL))
            i = i + 1

        i = 0
        intensity_fail_count = 0
        while (i < len(cloud.intensities) and i < len(TARGET_INTENSITIES)):
            d = cloud.intensities[i] - TARGET_INTENSITIES[i]
            if cloud.intensities[i] > 0:
                d = d/cloud.intensities[i]
            if cloud.intensities[i] < 0:
                intensity_fail_count += 1
                print("intensity_fail_count:" + str(intensity_fail_count) + " failed. intensity <0:" + str(cloud.intensiteis[i]))
            else:
                if ((d < - ERROR_TOL) or (d > ERROR_TOL)):
                    intensity_fail_count += 1
                    print("intensity_fail_count:" + str(intensity_fail_count) + " failed. error:" + str(d) + " exceeded tolerance:" + str(ERROR_TOL))
            i = i + 1

        if range_fail_count > FAIL_COUNT_TOL:
            print("Range fail count too large (" + str(range_fail_count) + "), failing scan")
            return

        if intensity_fail_count > FAIL_COUNT_TOL:
            print("Intensity fail count too large (" + str(intensity_fail_count) + "), failing scan")
            return

        self.success = True
    
    def test_scan(self):
        print("LNK\n")
        rospy.Subscriber("/base_scan", LaserScan, self.pointInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_DURATION
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], PointCloudTest, sys.argv) #, text_mode=True)


