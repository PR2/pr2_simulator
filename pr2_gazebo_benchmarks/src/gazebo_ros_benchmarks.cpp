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
 * SVN info: $Id$
 */


#include <pr2_gazebo_benchmarks/gazebo_ros_benchmarks.h>

#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <algorithm>
#include <assert.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_PLUGIN("gazebo_ros_benchmarks", GazeboRosBenchmarks);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosBenchmarks::GazeboRosBenchmarks()
    : Plugin()
{
  this->timing_initialized_                                = false;
  this->world_update_ros_connect_count_                    = 0;
  this->simulation_cycle_ros_connect_count_                = 0;
  this->world_update_start_sim_time_                       = 0;
  this->world_update_start_wall_time_                      = 0;
  this->world_update_end_sim_time_                         = 0;
  this->world_update_end_wall_time_                        = 0;
  this->last_world_update_start_sim_time_                  = 0;
  this->last_world_update_start_wall_time_                 = 0;
  this->last_world_update_end_sim_time_                    = 0;
  this->last_world_update_end_wall_time_                   = 0;
  std_srvs::Empty emp;
  this->ResetTimingStatistics(emp.request,emp.response);
}

bool GazeboRosBenchmarks::ResetTimingStatistics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
  this->world_update_pr2_gazebo_benchmarks_.max_duration       = -1.0;
  this->world_update_pr2_gazebo_benchmarks_.min_duration       = 1e6;
  this->world_update_pr2_gazebo_benchmarks_.avg_duration       = 0.0;
  this->world_update_pr2_gazebo_benchmarks_.tot_duration       = 0;
  this->world_update_pr2_gazebo_benchmarks_.cur_duration       = 0;
  this->world_update_pr2_gazebo_benchmarks_.count              = 0;
  this->simulation_cycle_pr2_gazebo_benchmarks_.max_duration   = -1.0;
  this->simulation_cycle_pr2_gazebo_benchmarks_.min_duration   = 1e6;
  this->simulation_cycle_pr2_gazebo_benchmarks_.avg_duration   = 0.0;
  this->simulation_cycle_pr2_gazebo_benchmarks_.tot_duration   = 0;
  this->simulation_cycle_pr2_gazebo_benchmarks_.cur_duration   = 0;
  this->simulation_cycle_pr2_gazebo_benchmarks_.count          = 0;
  return true;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBenchmarks::~GazeboRosBenchmarks()
{
  World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboRosBenchmarks::UpdateCBWorldUpdateStart, this));
  World::Instance()->DisconnectWorldUpdateEndSignal(boost::bind(&GazeboRosBenchmarks::UpdateCBWorldUpdateEnd, this));
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBenchmarks::Load()
{
  // start ros node if not initialized
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }
  this->rosnode_ = new ros::NodeHandle("");

  // start custom queue for ros
  this->ros_callback_queue_thread_ = new boost::thread( boost::bind( &GazeboRosBenchmarks::RosQueueThread,this ) );

  // publish timing info
  ros::AdvertiseOptions simulation_cycle_ros_ao = ros::AdvertiseOptions::create<pr2_gazebo_benchmarks::GazeboBenchmarks>(
    "simulation_cycle_benchmarks",1,
    boost::bind( &GazeboRosBenchmarks::SimulationCycleRosConnect,this),
    boost::bind( &GazeboRosBenchmarks::SimulationCycleRosDisconnect,this), ros::VoidPtr(), &this->ros_queue_);
  this->simulation_cycle_pub_ = this->rosnode_->advertise(simulation_cycle_ros_ao);

  ros::AdvertiseOptions world_update_ros_ao = ros::AdvertiseOptions::create<pr2_gazebo_benchmarks::GazeboBenchmarks>(
    "world_update_benchmarks",1,
    boost::bind( &GazeboRosBenchmarks::WorldUpdateRosConnect,this),
    boost::bind( &GazeboRosBenchmarks::WorldUpdateRosDisconnect,this), ros::VoidPtr(), &this->ros_queue_);
  this->world_update_pub_ = this->rosnode_->advertise(world_update_ros_ao);


  // Advertise more services on the custom queue
  ros::AdvertiseServiceOptions reset_timing_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      "reset_simulation_timing_statistics",boost::bind(&GazeboRosBenchmarks::ResetTimingStatistics,this,_1,_2),
      ros::VoidPtr(), &this->ros_queue_);
  this->reset_timing_service_ = this->rosnode_->advertiseService(reset_timing_aso);


  // Update the controller through callback slots
  World::Instance()->ConnectWorldUpdateStartSignal(boost::bind(&GazeboRosBenchmarks::UpdateCBWorldUpdateStart, this));
  World::Instance()->ConnectWorldUpdateEndSignal(boost::bind(&GazeboRosBenchmarks::UpdateCBWorldUpdateEnd, this));

}


////////////////////////////////////////////////////////////////////////////////
// Update the controller through callback slots
void GazeboRosBenchmarks::UpdateCBWorldUpdateStart()
{
    // get times
    this->world_update_start_sim_time_ = Simulator::Instance()->GetSimTime();
    this->world_update_start_wall_time_ = Simulator::Instance()->GetWallTime();
    //ROS_INFO("real: %f sim: %f",this->world_update_start_wall_time_.Double(),this->world_update_start_sim_time_.Double());

    // compute statistics
    if (this->timing_initialized_)
    {
      // keep track of simulation cycle time
      this->simulation_cycle_pr2_gazebo_benchmarks_.cur_duration = this->world_update_start_wall_time_.Double()
                                                              -this->last_world_update_start_wall_time_.Double();
      this->simulation_cycle_pr2_gazebo_benchmarks_.tot_duration += this->simulation_cycle_pr2_gazebo_benchmarks_.cur_duration;
      this->simulation_cycle_pr2_gazebo_benchmarks_.count++;
      this->simulation_cycle_pr2_gazebo_benchmarks_.avg_duration = this->simulation_cycle_pr2_gazebo_benchmarks_.tot_duration
                                                               /(double)this->simulation_cycle_pr2_gazebo_benchmarks_.count;
      if (this->simulation_cycle_pr2_gazebo_benchmarks_.max_duration < this->simulation_cycle_pr2_gazebo_benchmarks_.cur_duration)
        this->simulation_cycle_pr2_gazebo_benchmarks_.max_duration = this->simulation_cycle_pr2_gazebo_benchmarks_.cur_duration;
      if (this->simulation_cycle_pr2_gazebo_benchmarks_.min_duration > this->simulation_cycle_pr2_gazebo_benchmarks_.cur_duration)
        this->simulation_cycle_pr2_gazebo_benchmarks_.min_duration = this->simulation_cycle_pr2_gazebo_benchmarks_.cur_duration;
      if (this->simulation_cycle_ros_connect_count_>0)
        this->simulation_cycle_pub_.publish(this->simulation_cycle_pr2_gazebo_benchmarks_);
    }
    this->last_world_update_start_sim_time_  = this->world_update_start_sim_time_ ;
    this->last_world_update_start_wall_time_ = this->world_update_start_wall_time_;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller through callback slots
void GazeboRosBenchmarks::UpdateCBWorldUpdateEnd()
{
    // get times
    this->world_update_end_sim_time_ = Simulator::Instance()->GetSimTime();
    this->world_update_end_wall_time_ = Simulator::Instance()->GetWallTime();
    //ROS_INFO("real: %f sim: %f",this->world_update_end_wall_time_.Double(),this->world_update_end_sim_time_.Double());

    // compute statistics

    // compute statistics
    if (this->timing_initialized_)
    {
      // keep track of world update duration
      this->world_update_pr2_gazebo_benchmarks_.cur_duration = this->world_update_start_wall_time_.Double()
                                                          -this->world_update_end_wall_time_.Double();
      this->world_update_pr2_gazebo_benchmarks_.tot_duration += this->world_update_pr2_gazebo_benchmarks_.cur_duration;
      this->world_update_pr2_gazebo_benchmarks_.count++;
      this->world_update_pr2_gazebo_benchmarks_.avg_duration = this->world_update_pr2_gazebo_benchmarks_.tot_duration
                                                               /(double)this->world_update_pr2_gazebo_benchmarks_.count;
      if (this->world_update_pr2_gazebo_benchmarks_.max_duration < this->world_update_pr2_gazebo_benchmarks_.cur_duration)
        this->world_update_pr2_gazebo_benchmarks_.max_duration = this->world_update_pr2_gazebo_benchmarks_.cur_duration;
      if (this->world_update_pr2_gazebo_benchmarks_.min_duration > this->world_update_pr2_gazebo_benchmarks_.cur_duration)
        this->world_update_pr2_gazebo_benchmarks_.min_duration = this->world_update_pr2_gazebo_benchmarks_.cur_duration;
      if (this->world_update_ros_connect_count_>0)
        this->world_update_pub_.publish(this->world_update_pr2_gazebo_benchmarks_);
    }
    else
    {
      // do not update statistics yet this cycle
      this->timing_initialized_ = true;
    }

    this->last_world_update_end_sim_time_  = this->world_update_end_sim_time_ ;
    this->last_world_update_end_wall_time_ = this->world_update_end_wall_time_;

}

////////////////////////////////////////////////////////////////////////////////
// ROS Publisher
void GazeboRosBenchmarks::RosQueueThread()
{
  static const double timeout = 0.001;

  while (this->rosnode_->ok())
  {
    this->ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}


