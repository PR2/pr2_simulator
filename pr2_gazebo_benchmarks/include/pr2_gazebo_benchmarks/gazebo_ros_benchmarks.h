/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: Plugin for testing pr2 simulator performance
 * Author: John Hsu
 * Date: 1 July 2010
 * SVN: $Id$
 */
#ifndef GAZEBO_ROS_BENCHMARKS_HH
#define GAZEBO_ROS_BENCHMARKS_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo/gazeboserver.hh>
#include <pr2_gazebo_benchmarks/GazeboBenchmarks.h>
#include "std_srvs/Empty.h"

#include "boost/thread/mutex.hpp"

namespace gazebo
{

class GazeboRosBenchmarks : public Plugin
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosBenchmarks();

  /// \brief Destructor
  public: ~GazeboRosBenchmarks();

  /// \brief Load the controller
  /// \param node XML config node
  protected: void Load();

  /// \brief Update the controller
  protected: void UpdateCBWorldUpdateStart();
  protected: void UpdateCBWorldUpdateEnd();

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  /// \brief publish queue
  private: ros::CallbackQueue ros_queue_;
  private: void RosQueueThread();
  private: boost::thread* ros_callback_queue_thread_;

  /// \brief timing
  private: bool timing_initialized_;
  private: gazebo::Time world_update_start_sim_time_;
  private: gazebo::Time world_update_start_wall_time_;
  private: gazebo::Time world_update_end_sim_time_;
  private: gazebo::Time world_update_end_wall_time_;

  private: gazebo::Time last_world_update_start_sim_time_;
  private: gazebo::Time last_world_update_start_wall_time_;
  private: gazebo::Time last_world_update_end_sim_time_;
  private: gazebo::Time last_world_update_end_wall_time_;

  /// \brief world update step timing statistics
  private: pr2_gazebo_benchmarks::GazeboBenchmarks world_update_pr2_gazebo_benchmarks_;
  private: ros::Publisher world_update_pub_;
  /// \brief Keep track of number of connctions
  private: int world_update_ros_connect_count_;
  private: void WorldUpdateRosConnect() {this->world_update_ros_connect_count_++;};
  private: void WorldUpdateRosDisconnect() {this->world_update_ros_connect_count_--;};

  /// \brief simulation cycle timing statistics
  private: pr2_gazebo_benchmarks::GazeboBenchmarks simulation_cycle_pr2_gazebo_benchmarks_;
  private: ros::Publisher simulation_cycle_pub_;
  /// \brief Keep track of number of connctions
  private: int simulation_cycle_ros_connect_count_;
  private: void SimulationCycleRosConnect() {this->simulation_cycle_ros_connect_count_++;};
  private: void SimulationCycleRosDisconnect() {this->simulation_cycle_ros_connect_count_--;};


  /// \breif resest statistics
  private: ros::ServiceServer reset_timing_service_;
  private: bool ResetTimingStatistics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);


};

/** \} */
/// @}

}
#endif

