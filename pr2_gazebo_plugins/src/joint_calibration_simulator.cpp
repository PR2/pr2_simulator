/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Wim Meeussen

#include "pr2_gazebo_plugins/joint_calibration_simulator.h"
#include <angles/angles.h>


namespace gazebo {


JointCalibrationSimulator::JointCalibrationSimulator(boost::shared_ptr<const urdf::Joint> jnt)
  : calibration_rising_edge_valid_(false), 
    calibration_falling_edge_valid_(false),
    has_rising_(false), 
    has_falling_(false),
    continuous_(false),
    jnt_(jnt)
{
  // read calibration field
  if (jnt_->calibration){
    if (jnt_->calibration->rising){
      has_rising_ = true;
      rising_ = *(jnt_->calibration->rising);
    }
    if (jnt_->calibration->falling){
      has_falling_ = true;
      falling_ = *(jnt_->calibration->falling);
    }
  }
  

  // continuous joints
  if (jnt_->type == urdf::Joint::CONTINUOUS){
    continuous_ = true;
    if (has_rising_ && !has_falling_){
      has_falling_ = true;
      falling_ = rising_ + M_PI;
    }
    if (!has_rising_ && has_falling_){
      has_rising_ = true;
      rising_ = falling_ + M_PI;
    }
    rising_ = angles::two_pi_complement(rising_);
    falling_ = angles::two_pi_complement(falling_);
    if (rising_ < falling_)
      bump_ = true;
    else
      bump_ = false;
  }


  // check
  if (jnt_->type != urdf::Joint::CONTINUOUS && has_rising_ && has_falling_)
      ROS_ERROR("Non-continuous joint with both rising and falling edge not supported");
}


void JointCalibrationSimulator::setJointPosition(double pos)
{
  // compute calibration reading
  if (continuous_){
    double pos_c = angles::two_pi_complement(pos);
    if (pos_c > rising_ && pos_c < falling_)
      calibration_reading_ = bump_;
    else
      calibration_reading_ = !bump_;
  }
  else{
    if (has_rising_)
      calibration_reading_ = pos > rising_;
    else if (has_falling_)
      calibration_reading_ = pos < falling_;
  }

  // store state
  old_calibration_reading_ = calibration_reading_;
  old_pos_ = pos;

  // initialize
  if (!initialized_){
    initialized_ = true;
    return;
  }

  // tripped calibration flag
  if (old_calibration_reading_ != calibration_reading_){
    if (calibration_reading_){
      calibration_rising_edge_valid_ = true;
      if (pos > old_pos_)
        last_calibration_rising_edge_ = rising_;
      else
        last_calibration_rising_edge_ = falling_;
    }
    else{
      calibration_falling_edge_valid_ = true;
      if (pos > old_pos_)
        last_calibration_falling_edge_ = falling_;
      else
        last_calibration_falling_edge_ = rising_;
    }
  }
}



} //namespace
