#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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


##\author Kevin Watts
##\brief Tests that URDF models in pr2_component_descriptions parse and are valid

PKG = 'pr2_gazebo'
TEST_PKG = 'test_urdf2model_parser'

import roslib; roslib.load_manifest(PKG)

import subprocess
import os, sys
import tempfile

import rostest, unittest

def test_urdf_xacro_file(path):
    if not os.path.exists(path):
        print >> sys.stderr, "Unable to find file %s." % path
        return False

    # Ignore some files
    if not path.endswith('urdf.xacro') or path.endswith('urdf'):
        return True

    urdf_file = tempfile.NamedTemporaryFile()
    # xacro
    xacro_cmd = 'rosrun xacro xacro.py %s > %s' % (path, urdf_file.name)
    p = subprocess.Popen(xacro_cmd, shell=True)
    p.communicate()
    if not p.returncode == 0:
        print >> sys.stderr, "Xacro command returned with code %d for file %s" % (p.returncode, path)
        return False

    # gazebo tools
    model_file = tempfile.NamedTemporaryFile()
    model_cmd = 'rosrun gazebo urdf2model -f %s -o %s' % (urdf_file.name, model_file.name)
    p = subprocess.Popen(model_cmd, shell=True)
    p.communicate()
    if not p.returncode == 0:
        print >> sys.stderr, "Gazebo tools command returned with code %d for file %s" % (p.returncode, path)
        return False

    urdf_file.close()
    model_file.close()
        
    return True

# Add test to test all files in pr2_component_descriptions
class URDFParser(unittest.TestCase):
    def setUp(self):
        test_pkg_path = roslib.packages.get_pkg_dir(TEST_PKG)
        robots = os.listdir(os.path.join(test_pkg_path, 'robots'))
        self.files = [ os.path.join(test_pkg_path, 'robots', r) for r in robots ]
        
    def test_urdf_files(self):
        rv = True
        for f in self.files:
            rv = test_urdf_xacro_file(f) and rv
        
        self.assert_(rv, "URDF files failed to parse!")


if __name__ == '__main__':
    rostest.unitrun(PKG, 'check_urdf_files', URDFParser)


    
