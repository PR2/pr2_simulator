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

#ifndef GAZEBO_BATTERY_H
#define GAZEBO_BATTERY_H

#include <vector>
#include <map>
#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <pr2_msgs/PowerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pr2_gazebo_plugins/PlugCommand.h>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

namespace gazebo
{
class XMLConfigNode;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup gazebo_battery GazeboBattery class

  \brief GazeboBattery Plugin
  
  This plugin simulates battery usage.
  GazeboBattery requires model as its parent.

  \verbatim
  <model:physical name="ray_model">
    <!-- GazeboBattery -->
    <controller:gazebo_battery name="gazebo_battery" plugin="libgazebo_battery.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1000.0</updateRate>
      <interface:audio name="gazebo_battery_dummy_iface" />
    </controller:gazebo_battery>
  </model:phyiscal>
  \endverbatim
 
\{


*/

/**
 * \brief Battery simulation
 *   \li starts a ROS node if none exists
 *   \li return power state and diagnostic message over ROS topic.
 * .
 *
  \verbatim
  <model:physical name="ray_model">
    <!-- GazeboBattery -->
    <controller:gazebo_battery name="gazebo_battery" plugin="libgazebo_battery.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1000.0</updateRate>
      <interface:audio name="gazebo_battery_dummy_iface" />
    </controller:gazebo_battery>
  </model:phyiscal>
  \endverbatim
**/


class GazeboBattery : public gazebo::Controller
{
public:
  GazeboBattery(Entity *parent);
  virtual ~GazeboBattery();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  Model *parent_model_;

  /// \brief ros message for power state
  pr2_msgs::PowerState power_state_;
  /// \brief ros message for battery state
  pr2_msgs::PowerState battery_state_;

  /// \brief ros message for diagnostic messages
  diagnostic_msgs::DiagnosticArray diagnostic_message_;
  diagnostic_msgs::DiagnosticStatus diagnostic_status_;

  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;
  private: ros::Subscriber sub_;

  /// \brief power state topic name
  private: std::string stateTopicName_;

  /// \brief diag. msg. topic name
  private: std::string diagnosticMessageTopicName_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock_;

  /// \brief stores current simulator time
  private: double current_time_;

  /// \brief stores last simulator time
  private: double last_time_;

  /// \brief rate to broadcast diagnostic message
  private: double diagnostic_rate_;

  /// \brief rate to broadcast power states message
  private: double power_state_rate_;

  /// \brief some internal variables for keeping track of simulated battery
  ///           @todo make consumption rate vary with joint commands, motion, etc

  /// \brief full capacity of battery
  private: double full_capacity_;

  /// \brief charge state;
  private: double charge_;

  /// \brief default charge rate when plugged in
  private: double default_charge_rate_;

  /// \brief power drain, if this is negative, we are charging the battery.
  private: double consumption_rate_;
  private: double default_consumption_rate_;

  /// \brief listen to ROS to see if we are charging
  private: void SetPlug(const pr2_gazebo_plugins::PlugCommandConstPtr& plug_msg);

};

/** \} */
/// @}

}

#endif

