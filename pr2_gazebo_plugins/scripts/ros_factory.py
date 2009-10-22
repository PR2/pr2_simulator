#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_gazebo_plugins')

import sys
import string
import rospy
from pr2_gazebo_plugins.srv import SpawnModel
from pr2_gazebo_plugins.srv import DeleteModel
from geometry_msgs.msg import Pose

def spawn_model_client_from_urdf_param(model_name,param_name):
    rospy.wait_for_service('spawn_model')
    try:
        spawn_model= rospy.ServiceProxy('spawn_model', SpawnModel)
        resp1 = spawn_model(model_name,param_name,2,"/",Pose())
        return resp1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [options]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        model_name = sys.argv[1]
        param_name = sys.argv[2]
    else:
        print usage()
        sys.exit(1)
    success = spawn_model_client_from_urdf_param(model_name,param_name)
    print "test spawning success",success
