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
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
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
    virtual void UpdateChild();

private:
    virtual void LoadThread();
    virtual void ConnectCb();
    virtual void DisconnectCb();
    virtual void QueueThread();
    void SetPlug(const pr2_gazebo_plugins::PlugCommandConstPtr& plug_msg);

private:
    // parameters
    /// \brief namespace of robot in ROS system
    std::string robot_namespace_;
    /// \brief name of published topic
    std::string power_state_topic_;
    /// \brief rate to broadcast power state message
    double power_state_rate_;
    /// \brief full capacity of battery (Ah)
    double full_capacity_;
    /// \brief discharge rate when not plugged in (W)
    double discharge_rate_;
    /// \brief charge rate (W)
    double charge_rate_;
    /// \brief discharge voltage when plugged in (V)
    double discharge_voltage_;
    /// \brief charge voltage when plugged in (V)
    double charge_voltage_;

    // ros
    ros::NodeHandle* rosnode_;
    ros::Publisher   power_state_pub_;
    ros::Subscriber  plugged_in_sub_;
    boost::thread deferred_load_thread_;
    boost::thread callback_queue_thread_;
    ros::CallbackQueue queue_;
    int connect_count_;

    // gazebo
    event::ConnectionPtr update_connection_;
    physics::WorldPtr world_;
    sdf::ElementPtr sdf_;

    // internal variables
    /// \brief latest time on callback is called
    double last_time_;
    /// \brief charge state (Ah)
    double charge_;
    /// \brief voltage (V)
    double voltage_;
    /// \brief published messages
    pr2_msgs::PowerState power_state_;
    /// \brief lock access to fields that are used in message callbacks
    boost::mutex lock_;
};

}

#endif
