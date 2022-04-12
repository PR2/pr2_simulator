#!/usr/bin/env python
# Author: John Hsu

PKG = 'test_pr2_grasping'
NAME = 'test_joint_position_controllers'

import math
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
import actionlib
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose,Quaternion,Point, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from pr2_controllers_msgs.msg import Pr2GripperCommand, JointControllerState
from trajectory_msgs.msg import JointTrajectory
from actionlib_msgs.msg import GoalStatusArray
import tf.transformations as tft
from numpy import float64

def positionState(state):
  print("set_point : ",state.set_point)
  print("process_value : ",state.process_value)
  print("command : ",state.command)
  print("error : ",state.error)

def gripperState(state):
  print("set_point : ",state.set_point)
  print("process_value : ",state.process_value)
  print("command : ",state.command)
  print("error : ",state.error)

if __name__ == '__main__':

  sub_top = {}
  sub_top[0]  = "/l_elbow_flex_position_controller/state"    
  sub_top[1]  = "/l_forearm_roll_position_controller/state"  
  sub_top[2]  = "/l_gripper_position_controller/state"       
  sub_top[3]  = "/l_shoulder_lift_position_controller/state" 
  sub_top[4]  = "/l_shoulder_pan_position_controller/state"  
  sub_top[5]  = "/l_upper_arm_roll_position_controller/state"
  sub_top[6]  = "/l_wrist_flex_position_controller/state"    
  sub_top[7]  = "/l_wrist_roll_position_controller/state"    
  sub_top[8]  = "/r_elbow_flex_position_controller/state"    
  sub_top[9]  = "/r_forearm_roll_position_controller/state"  
  sub_top[10] = "/r_gripper_position_controller/state"       
  sub_top[11] = "/r_shoulder_lift_position_controller/state" 
  sub_top[12] = "/r_shoulder_pan_position_controller/state"  
  sub_top[13] = "/r_upper_arm_roll_position_controller/state"
  sub_top[14] = "/r_wrist_flex_position_controller/state"    
  sub_top[15] = "/r_wrist_roll_position_controller/state"    
  sub_top[16] = "/torso_lift_position_controller/state"      
  sub_top[17] = "/l_gripper_controller/state"                
  sub_top[18] = "/r_gripper_controller/state"                
  sub_top[19] = "/torso_controller/state"                    

  pub_top = {}
  pub_top[0]  = "/l_elbow_flex_position_controller/command"    
  pub_top[1]  = "/l_forearm_roll_position_controller/command"  
  pub_top[2]  = "/l_gripper_position_controller/command"       
  pub_top[3]  = "/l_shoulder_lift_position_controller/command" 
  pub_top[4]  = "/l_shoulder_pan_position_controller/command"  
  pub_top[5]  = "/l_upper_arm_roll_position_controller/command"
  pub_top[6]  = "/l_wrist_flex_position_controller/command"    
  pub_top[7]  = "/l_wrist_roll_position_controller/command"    
  pub_top[8]  = "/r_elbow_flex_position_controller/command"    
  pub_top[9]  = "/r_forearm_roll_position_controller/command"  
  pub_top[10] = "/r_gripper_position_controller/command"       
  pub_top[11] = "/r_shoulder_lift_position_controller/command" 
  pub_top[12] = "/r_shoulder_pan_position_controller/command"  
  pub_top[13] = "/r_upper_arm_roll_position_controller/command"
  pub_top[14] = "/r_wrist_flex_position_controller/command"    
  pub_top[15] = "/r_wrist_roll_position_controller/command"    
  pub_top[16] = "/torso_lift_position_controller/command"      
  pub_top[17] = "/l_gripper_controller/command"                
  pub_top[18] = "/r_gripper_controller/command"                
  pub_top[19] = "/torso_controller/command"                    

  pub_cmd = {}
  pub_cmd[0]  = rospy.Publisher(pub_top[0]  , Float64)
  pub_cmd[1]  = rospy.Publisher(pub_top[1]  , Float64)
  pub_cmd[2]  = rospy.Publisher(pub_top[2]  , Float64)
  pub_cmd[3]  = rospy.Publisher(pub_top[3]  , Float64)
  pub_cmd[4]  = rospy.Publisher(pub_top[4]  , Float64)
  pub_cmd[5]  = rospy.Publisher(pub_top[5]  , Float64)
  pub_cmd[6]  = rospy.Publisher(pub_top[6]  , Float64)
  pub_cmd[7]  = rospy.Publisher(pub_top[7]  , Float64)
  pub_cmd[8]  = rospy.Publisher(pub_top[8]  , Float64)
  pub_cmd[9]  = rospy.Publisher(pub_top[9]  , Float64)
  pub_cmd[10] = rospy.Publisher(pub_top[10] , Float64)
  pub_cmd[11] = rospy.Publisher(pub_top[11] , Float64)
  pub_cmd[12] = rospy.Publisher(pub_top[12] , Float64)
  pub_cmd[13] = rospy.Publisher(pub_top[13] , Float64)
  pub_cmd[14] = rospy.Publisher(pub_top[14] , Float64)
  pub_cmd[15] = rospy.Publisher(pub_top[15] , Float64)
  pub_cmd[16] = rospy.Publisher(pub_top[16] , Float64)
  pub_cmd[17] = rospy.Publisher(pub_top[17] , Pr2GripperCommand)
  pub_cmd[18] = rospy.Publisher(pub_top[18] , Pr2GripperCommand)
  pub_cmd[19] = rospy.Publisher(pub_top[19] , JointTrajectory)

  rospy.init_node(NAME, anonymous=False)

  rospy.Subscriber(sub_top[0]  , JointControllerState, positionState)

  cmd = Float64()
  cmd.data = -0.3

  timeout = 10.0
  while True:
    cmd.data = cmd.data * -1
    start_time = time.time()
    end_time = start_time + timeout

    while time.time() < end_time:
      pub_cmd[0] .publish( cmd )
      pub_cmd[1] .publish( cmd )
      pub_cmd[2] .publish( cmd )
      pub_cmd[3] .publish( cmd )
      pub_cmd[4] .publish( cmd )
      pub_cmd[5] .publish( cmd )
      pub_cmd[6] .publish( cmd )
      pub_cmd[7] .publish( cmd )
      pub_cmd[8] .publish( cmd )
      pub_cmd[9] .publish( cmd )
      pub_cmd[10].publish( cmd )
      pub_cmd[11].publish( cmd )
      pub_cmd[12].publish( cmd )
      pub_cmd[13].publish( cmd )
      pub_cmd[14].publish( cmd )
      pub_cmd[15].publish( cmd )
      pub_cmd[16].publish( cmd )
      #pub_cmd[17].publish(0)
      #pub_cmd[18].publish(0)
      #pub_cmd[19].publish(0)
      time.sleep(0.1)
