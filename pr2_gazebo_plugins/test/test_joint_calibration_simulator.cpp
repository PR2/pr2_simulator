/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <string>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include "pr2_gazebo_plugins/joint_calibration_simulator.h"

using namespace gazebo;

int g_argc;
char** g_argv;

class TestParser : public testing::Test
{
public:

  /// constructor
  TestParser() {}

  /// Destructor
  ~TestParser() {}
};




TEST_F(TestParser, test)
{
  ASSERT_TRUE(g_argc == 2);

  urdf::Model robot;
  ASSERT_TRUE(robot.initFile(g_argv[1]));

  boost::shared_ptr<const urdf::Joint> jnt_cont1 = robot.getJoint("continuous1");
  boost::shared_ptr<const urdf::Joint> jnt_cont2 = robot.getJoint("continuous2");
  boost::shared_ptr<const urdf::Joint> jnt_rev  = robot.getJoint("revolute");
  ASSERT_TRUE(jnt_cont1 != NULL);
  ASSERT_TRUE(jnt_cont2 != NULL);
  ASSERT_TRUE(jnt_rev != NULL);

  JointCalibrationSimulator jnt_sim_cont1(jnt_cont1);
  JointCalibrationSimulator jnt_sim_cont2(jnt_cont2);
  JointCalibrationSimulator jnt_sim_rev(jnt_rev);
  ASSERT_FALSE(jnt_sim_cont1.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont1.calibration_falling_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont2.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont2.calibration_falling_edge_valid_);
  ASSERT_FALSE(jnt_sim_rev.calibration_falling_edge_valid_);
  ASSERT_FALSE(jnt_sim_rev.calibration_falling_edge_valid_);

  // test cont1
  jnt_sim_cont1.setJointPosition(0.01);
  ASSERT_FALSE(jnt_sim_cont1.calibration_reading_);
  ASSERT_FALSE(jnt_sim_cont1.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont1.calibration_falling_edge_valid_);

  jnt_sim_cont1.setJointPosition(1.0);
  ASSERT_FALSE(jnt_sim_cont1.calibration_reading_);
  ASSERT_FALSE(jnt_sim_cont1.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont1.calibration_falling_edge_valid_);

  jnt_sim_cont1.setJointPosition(1.6);
  ASSERT_TRUE(jnt_sim_cont1.calibration_reading_);
  ASSERT_TRUE(jnt_sim_cont1.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont1.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_cont1.last_calibration_rising_edge_, 1.5);

  jnt_sim_cont1.setJointPosition(1.4);
  ASSERT_FALSE(jnt_sim_cont1.calibration_reading_);
  ASSERT_TRUE(jnt_sim_cont1.calibration_rising_edge_valid_);
  ASSERT_TRUE(jnt_sim_cont1.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_cont1.last_calibration_falling_edge_, 1.5);  

  jnt_sim_cont1.setJointPosition(-0.1);
  ASSERT_TRUE(jnt_sim_cont1.calibration_reading_);
  ASSERT_TRUE(jnt_sim_cont1.calibration_rising_edge_valid_);
  ASSERT_TRUE(jnt_sim_cont1.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_cont1.last_calibration_rising_edge_, 0.0);  

  // test cont2
  jnt_sim_cont2.setJointPosition(0.01);
  ASSERT_TRUE(jnt_sim_cont2.calibration_reading_);
  ASSERT_FALSE(jnt_sim_cont2.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont2.calibration_falling_edge_valid_);

  jnt_sim_cont2.setJointPosition(1.0);
  ASSERT_TRUE(jnt_sim_cont2.calibration_reading_);
  ASSERT_FALSE(jnt_sim_cont2.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_cont2.calibration_falling_edge_valid_);

  jnt_sim_cont2.setJointPosition(1.6);
  ASSERT_FALSE(jnt_sim_cont2.calibration_reading_);
  ASSERT_FALSE(jnt_sim_cont2.calibration_rising_edge_valid_);
  ASSERT_TRUE(jnt_sim_cont2.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_cont2.last_calibration_falling_edge_, 1.5);

  jnt_sim_cont2.setJointPosition(1.4);
  ASSERT_TRUE(jnt_sim_cont2.calibration_reading_);
  ASSERT_TRUE(jnt_sim_cont2.calibration_rising_edge_valid_);
  ASSERT_TRUE(jnt_sim_cont2.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_cont2.last_calibration_rising_edge_, 1.5);  

  jnt_sim_cont2.setJointPosition(-0.1);
  ASSERT_FALSE(jnt_sim_cont2.calibration_reading_);
  ASSERT_TRUE(jnt_sim_cont2.calibration_rising_edge_valid_);
  ASSERT_TRUE(jnt_sim_cont2.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_cont2.last_calibration_falling_edge_, 0.0);  

  // test rev
  jnt_sim_rev.setJointPosition(0.01);
  ASSERT_FALSE(jnt_sim_rev.calibration_reading_);
  ASSERT_FALSE(jnt_sim_rev.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_rev.calibration_falling_edge_valid_);

  jnt_sim_rev.setJointPosition(0.4);
  ASSERT_FALSE(jnt_sim_rev.calibration_reading_);
  ASSERT_FALSE(jnt_sim_rev.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_rev.calibration_falling_edge_valid_);

  jnt_sim_rev.setJointPosition(1.6);
  ASSERT_TRUE(jnt_sim_rev.calibration_reading_);
  ASSERT_TRUE(jnt_sim_rev.calibration_rising_edge_valid_);
  ASSERT_FALSE(jnt_sim_rev.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_rev.last_calibration_rising_edge_, 0.5);

  jnt_sim_rev.setJointPosition(0.4);
  ASSERT_FALSE(jnt_sim_rev.calibration_reading_);
  ASSERT_TRUE(jnt_sim_rev.calibration_rising_edge_valid_);
  ASSERT_TRUE(jnt_sim_rev.calibration_falling_edge_valid_);
  ASSERT_EQ(jnt_sim_rev.last_calibration_falling_edge_, 0.5);  


  SUCCEED();
}




int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_joint_calibration_simulator");
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
