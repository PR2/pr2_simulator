#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Kaijen Hsiao

# Simulates the fingertip sensor array based on the bumper contact states
# subscribes to Gazebo's ContactsState messages
# publishes PressureState messages

import roslib
roslib.load_manifest('pr2_gazebo')
import rospy
import tf
from pr2_msgs.msg import PressureState
from geometry_msgs.msg import PointStamped, Vector3Stamped
from gazebo_plugins.msg import ContactState, ContactsState

import numpy
import math
import threading
import pdb
import time

from fingertip_pressure import fingertip_geometry

DEBUG = 0

#vector norm of numpy 3-vector (as array)
def vectnorm(vect):
    return math.sqrt(numpy.dot(vect, vect))

#vector norm of numpy array of vectors (nx3)
def arrayvectnorm(vectarray):
    try:
        return [vectnorm(x) for x in vectarray]
    except:
        pdb.set_trace()

#angle diff of two numpy 3-arrays
def anglediff(vect1, vect2):
    return math.acos(numpy.dot(vect1, vect2)/(vectnorm(vect1)*vectnorm(vect2)))

#pretty-print a list of floats to str
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])

#pretty-print a list of lists of floats to str
def pplistlist(list):
    return '\n'.join([pplist(x) for x in list])

#convert a Vector3 or a Point msg to a 4x1 numpy matrix
def vectorToNumpyMat(vector):
    return numpy.matrix([[vector.x, vector.y, vector.z, 1]]).transpose()



#fingertip array element info 
#(x toward tip, y away from front in l/toward front in r, z to the left in l/to the right in r)
class SensorArrayInfo:
    def __init__(self, tip):

        if tip == 'l':
            l_msg = fingertip_geometry.pressureInformation('', 1)
            (self.force_per_unit, self.center, self.halfside1, self.halfside2) = \
                (l_msg.force_per_unit, l_msg.center, l_msg.halfside1, l_msg.halfside2)
        else:
            r_msg = fingertip_geometry.pressureInformation('', -1)
            (self.force_per_unit, self.center, self.halfside1, self.halfside2) = \
                (r_msg.force_per_unit, r_msg.center, r_msg.halfside1, r_msg.halfside2)

        #convert to numpy arrays 
        self.center = [numpy.array([v.x, v.y, v.z]) for v in self.center]
        self.halfside1 = [numpy.array([v.x, v.y, v.z]) for v in self.halfside1]
        self.halfside2 = [numpy.array([v.x, v.y, v.z]) for v in self.halfside2]
        
        #compute sensor element normals
        self.normal = numpy.cross(self.halfside1, self.halfside2)
        self.normal = [self.normal[i] / vectnorm(self.normal[i]) for i in range(len(self.normal))]

        #print "tip:", tip
        #print "center:", self.center
        #print "halfside1:", self.halfside1
        #print "halfside2:", self.halfside2
        #print "normal:", self.normal


class contactArraySimulator:

    #gripper is 'r' or 'l'
    def __init__(self, gripper): 
        self.gripper = gripper

        self.sensor_array_info = {'r': SensorArrayInfo('r'), 'l': SensorArrayInfo('l')}

        self.lock = threading.Lock()

        #initialize the dictionaries for the contact positions, normals, and depths
        self.contact_positions = {'r':[], 'l':[]}
        self.contact_normals = {'r':[], 'l':[]}
        self.contact_depths = {'r':[], 'l':[]}

        #the frames for the fingertip contact array positions
        self.fingertip_frameid = {'r':gripper+'_gripper_r_finger_tip_link', 
                                  'l':gripper+'_gripper_l_finger_tip_link'}

        #subscribe to Gazebo fingertip contact info 
        rospy.Subscriber(gripper + "_gripper_l_finger_tip_bumper/state", ContactsState, self.l_contact_callback)
        rospy.Subscriber(gripper + "_gripper_r_finger_tip_bumper/state", ContactsState, self.r_contact_callback)

        #publish contact array info as PressureState messages, just like the real robot does
        self.pub = rospy.Publisher("pressure/"+gripper+"_gripper_motor", PressureState)



    #callback for receiving left fingertip ContactsState messages
    def l_contact_callback(self, bumperstate):
        self.store_contacts('l', bumperstate)

    #callback for receiving right fingertip ContactsState messages
    def r_contact_callback(self, bumperstate):
        self.store_contacts('r', bumperstate)

    #store the contact positions and normals in the fingertip frame
    def store_contacts(self, whichtip, bumperstate):

        #self.contact_positions[whichtip] = []
        #self.contact_normals[whichtip] = []
        #self.contact_depths[whichtip] = []

        fingertip_frameid = self.fingertip_frameid[whichtip]

        self.lock.acquire()        
        for contactstate in bumperstate.states:

            #transform each contact point and normal to the fingertip frame
            for i in range(len(contactstate.depths)):

                #account for map offset of (25.7, 25.7, 0)
                contact_pos = vectorToNumpyMat(contactstate.contact_positions[i])
                contact_normal = vectorToNumpyMat(contactstate.contact_normals[i])
                array_pos = numpy.array([contact_pos[0,0], contact_pos[1,0], contact_pos[2,0]])
                array_normal = numpy.array([contact_normal[0,0], contact_normal[1,0], contact_normal[2,0]])
                self.contact_positions[whichtip].append(array_pos)
                self.contact_normals[whichtip].append(array_normal)
                self.contact_depths[whichtip].append(contactstate.depths[i])

        self.lock.release()

        if DEBUG:
            print whichtip, "positions:\n", pplistlist(self.contact_positions[whichtip])
            print whichtip, "normals:\n", pplistlist(self.contact_normals[whichtip])
                                                   


    #publish the current simulated fingertip sensor array readings
    def publish(self):

        self.lock.acquire()

        #compute fingertip sensor readings from observed contacts 
        sensor_element_count = len(self.sensor_array_info['l'].center)
        finger_tip_readings = {'l':[0]*sensor_element_count, 'r':[0]*sensor_element_count}
        for tip in ['l','r']:
            for contactnum in range(len(self.contact_positions[tip])):

                #figure out which sensor array element the contact falls into, if any    
                nearest_array_element = 'not found'
                nearest_array_weighted_dist = 1e6;

                dists = []
                anglediffs = []
                weighteddists = []

                for arrayelem in range(sensor_element_count):
                    
                    center = self.sensor_array_info[tip].center[arrayelem]
                    halfside1 = self.sensor_array_info[tip].halfside1[arrayelem]
                    halfside2 = self.sensor_array_info[tip].halfside2[arrayelem]
                    normal = self.sensor_array_info[tip].normal[arrayelem]

                    sensedpoint = self.contact_positions[tip][contactnum]
                    sensednormal = self.contact_normals[tip][contactnum]
                    
                    posdiff = self.contact_positions[tip][contactnum] - \
                        self.sensor_array_info[tip].center[arrayelem]

                    #distance from sensor plane (magnitude of projection onto normal)
                    dist_from_plane = math.fabs(numpy.dot(posdiff, sensednormal)/vectnorm(sensednormal))
                    
                    #distance from sensor element rectangle edge (- is inside)
                    halfside1proj = numpy.dot(posdiff, halfside1)/vectnorm(halfside1)
                    halfside2proj = numpy.dot(posdiff, halfside2)/vectnorm(halfside2)
                    halfside1dist = math.fabs(halfside1proj) - vectnorm(halfside1)
                    halfside2dist = math.fabs(halfside2proj) - vectnorm(halfside2)
                    
                    #find the euclidean dist from the sensor element pad
                    if halfside1dist < 0: 
                        halfside1dist = 0.
                    if halfside2dist < 0:
                        halfside2dist = 0.
                    dist = math.sqrt(halfside1dist**2+halfside2dist**2+dist_from_plane**2)
                    dists.append(dist)

                    #has to be at least near the element 
                    #(within 1 cm; interpenetration can make positions weird)
                    if dist > .01 and not DEBUG:
                        continue

                    #angle difference between the two normals
                    normal_angle_diff = anglediff(sensednormal, normal)
                    anglediffs.append(normal_angle_diff)

                    #weight the euclidean distance difference with the normal distance 
                    # (1.0 cm = 1 rad; don't want to be on the wrong side, but friction can
                    # make the angles weird) 
                    weighted_dist = normal_angle_diff + dist / .01
                    weighteddists.append(weighted_dist)

                    if weighted_dist < nearest_array_weighted_dist:
                        nearest_array_weighted_dist = weighted_dist
                        nearest_array_element = arrayelem
                
                if DEBUG:
                    print "tip:", tip
                    print "dists:", pplist(dists)
                    print "anglediffs:", pplist(anglediffs)
                    print "weighteddists:", pplist(weighteddists)

                #update that sensor element's depth if this depth is greater
                #(the maximum readings around 10000?)
                if nearest_array_element != 'not found':
                    if DEBUG:
                        print "nearest_array_element:", nearest_array_element
                        print "nearest_array_weighted_dist: %0.3f"%nearest_array_weighted_dist
                    newreading = self.contact_depths[tip][contactnum] * \
                        self.sensor_array_info[tip].force_per_unit[nearest_array_element] * 5000.
                    if newreading > 10000:
                        newreading = 10000

                    if newreading > finger_tip_readings[tip][nearest_array_element]:
                        finger_tip_readings[tip][nearest_array_element] = newreading

            self.contact_positions[tip] = []
            self.contact_normals[tip] = []
            self.contact_depths[tip] = []

        self.lock.release()

        if DEBUG:
            print "left tip readings:", pplist(finger_tip_readings['l'])
            print "right tip readings:", pplist(finger_tip_readings['r'])

        #fill in the message and publish it
        ps = PressureState()
        ps.header.stamp = rospy.get_rostime();
        ps.l_finger_tip = []
        ps.r_finger_tip = []
        for i in range(sensor_element_count):
            ps.l_finger_tip.append(finger_tip_readings['l'][i])
            ps.r_finger_tip.append(finger_tip_readings['r'][i])
        self.pub.publish(ps)


            

if __name__ == '__main__':

    rospy.init_node('sim_contact_translator', anonymous=True)

    s1 = contactArraySimulator('r')
    s2 = contactArraySimulator('l')
        
    print "done initializing, starting to publish"

    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        s1.publish()
        s2.publish()
