#! /usr/bin/python
import roslib
roslib.load_manifest('pr2_gazebo_benchmarks')

import sys
import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

rospy.init_node('single_joint_position', anonymous=True)
gripper_action_topic = sys.argv[1]+'_gripper_controller/gripper_action'
client = actionlib.SimpleActionClient(gripper_action_topic,
                                      Pr2GripperCommandAction)
client.wait_for_server()

goal = Pr2GripperCommandGoal();
goal.command.position = float(sys.argv[2])
goal.command.max_effort = float(sys.argv[3])
client.send_goal(goal)
client.wait_for_result()
if client.get_state() == GoalStatus.SUCCEEDED:
    print "Success"
