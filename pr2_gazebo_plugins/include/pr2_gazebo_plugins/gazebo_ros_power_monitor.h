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

#ifndef PR2_GAZEBO_PLUGINS_ROS_POWER_MONITOR_H
#define PR2_GAZEBO_PLUGINS_ROS_POWER_MONITOR_H

#include <map>
#include <vector>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <pr2_msgs/PowerState.h>
#include <pr2_gazebo_plugins/PlugCommand.h>

namespace gazebo {

class GazeboRosPowerMonitor : public ModelPlugin
{
public:
    GazeboRosPowerMonitor();
    virtual ~GazeboRosPowerMonitor();

protected:
    // Inherited from Controller
    void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
    virtual void InitChild();
    virtual void UpdateChild();

private:
    /// \brief listen to ROS to see if we are charging
    void SetPlug(const pr2_gazebo_plugins::PlugCommandConstPtr& plug_msg);

private:
    gazebo::physics::ModelPtr parent_model_;
    double curr_time_;
    double last_time_;

    //ParamT<std::string>* robot_namespace_param_;
    //ParamT<std::string>* power_state_topic_param_;
    std::string robot_namespace_;
    std::string power_state_topic_;

    ros::NodeHandle* rosnode_;
    ros::Subscriber  plugged_in_sub_;
    ros::Publisher   power_state_pub_;

    /// \brief lock access to fields that are used in message callbacks
    boost::mutex lock_;

    pr2_msgs::PowerState power_state_;

    /// \brief rate to broadcast power state message
    //ParamT<double>* power_state_rate_param_;
    double power_state_rate_;

    /// \brief internal variables for keeping track of simulated battery

    /// \brief full capacity of battery (Ah)
    //ParamT<double>* full_capacity_param_;
    double full_capacity_;

    /// \brief charge rate when plugged in (W)
    //ParamT<double>* charge_rate_param_;

    /// \brief discharge rate when not plugged in (W)
    //ParamT<double>* discharge_rate_param_;
    double discharge_rate_;

    /// \brief charge voltage when plugged in (V)
    //ParamT<double>* charge_voltage_param_;
    double charge_voltage_;

    /// \brief discharge voltage when plugged in (V)
    //ParamT<double>* discharge_voltage_param_;
    double discharge_voltage_;

    /// \brief charge state (Ah)
    double charge_;

    /// \brief charge rate (W)
    double charge_rate_;

    /// \brief voltage (V)
    double voltage_;

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
