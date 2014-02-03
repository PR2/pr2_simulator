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

#ifndef GAZEBO_CONTROLLER_MANAGER_H
#define GAZEBO_CONTROLLER_MANAGER_H

#include <vector>
#include <map>

#include <tinyxml.h>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_controller_manager/controller_manager.h>
#include <pr2_gazebo_plugins/SetModelsJointsStates.h>
#include <pr2_mechanism_model/robot.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>

#undef USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#endif

namespace gazebo
{

class GazeboRosControllerManager : public ModelPlugin
{
public:
  GazeboRosControllerManager();
  virtual ~GazeboRosControllerManager();
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

protected:
  // Inherited from gazebo::Controller
  virtual void UpdateChild();

private:

  gazebo::physics::ModelPtr parent_model_;
  pr2_hardware_interface::HardwareInterface hw_;
  pr2_controller_manager::ControllerManager *cm_;

  /// @todo The fake state helps Gazebo run the transmissions backwards, so
  ///       that it can figure out what its joints should do based on the
  ///       actuator values.
  pr2_mechanism_model::RobotState *fake_state_;
  std::vector<gazebo::physics::JointPtr>  joints_;

  /// \brief Service Call Name
  //private: ParamT<std::string> *setModelsJointsStatesServiceNameP;
  //private: std::string setModelsJointsStatesServiceName;

  /*
   * \brief read pr2.xml for actuators, and pass tinyxml node to mechanism control node's initXml.
   */
  void ReadPr2Xml();

  /*
   *  \brief pointer to ros node
   */
  ros::NodeHandle* rosnode_;

  /// \brief ros service
  private: ros::ServiceServer setModelsJointsStatesService;

  ///\brief ros service callback
  private: bool setModelsJointsStates(pr2_gazebo_plugins::SetModelsJointsStates::Request &req,
                                      pr2_gazebo_plugins::SetModelsJointsStates::Response &res);

  ///\brief ros service callback
  /*
   *  \brief tmp vars for performance checking
   */
  double wall_start_, sim_start_;

  /// \brief set topic name of robot description parameter
  //ParamT<std::string> *robotParamP;
  //ParamT<std::string> *robotNamespaceP;
  std::string robotParam;
  std::string robotNamespace;

  bool fake_calibration_;

#ifdef USE_CBQ
  private: ros::CallbackQueue controller_manager_queue_;
  private: void ControllerManagerQueueThread();
  private: boost::thread controller_manager_callback_queue_thread_;
#endif
  private: void ControllerManagerROSThread();
  private: boost::thread ros_spinner_thread_;

  // Pointer to the model
  private: physics::WorldPtr world;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  // subscribe to world stats
  private: transport::NodePtr node;
  private: transport::SubscriberPtr statsSub;
  private: common::Time simTime;

};

}

#endif

