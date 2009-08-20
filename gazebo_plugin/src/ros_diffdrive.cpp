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
 * Desc: Controller for diffdrive robots in gazebo.
 * Author: Tony Pratkanis
 * Date: 17 April 2009
 * SVN info: $Id$
 */

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <boost/bind.hpp>

class DiffDrive {
public:
  gazebo::PositionIface *posIface;
  ros::NodeHandle* rnh_;
  ros::Subscriber  sub_;
  ros::Publisher   pub_;
  
  void cmdVelCallBack(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
        std::cout << " pos " << this->posIface
                  <<    " x " << cmd_msg->linear.x
                  <<    " y " << cmd_msg->linear.y
                  <<    " z " << cmd_msg->angular.z
                  << std::endl;
        
    if (this->posIface) {
      this->posIface->Lock(1);
      this->posIface->data->cmdVelocity.pos.x = cmd_msg->linear.x;
      this->posIface->data->cmdVelocity.pos.y = cmd_msg->linear.y;
      this->posIface->data->cmdVelocity.yaw = cmd_msg->angular.z;
      this->posIface->Unlock();
    }
  }

  DiffDrive() {
    gazebo::Client *client = new gazebo::Client();
    gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
    this->posIface = new gazebo::PositionIface();
  
    int serverId = 0;
    
    /// Connect to the libgazebo server
    try {
      client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
    } catch (gazebo::GazeboError e) {
      std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
      return;
    }
    
    /// Open the Simulation Interface
    try {
      simIface->Open(client, "default");
    } catch (gazebo::GazeboError e) {
      std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
      return;
    }
    
    /// Open the Position interface
    try {
      this->posIface->Open(client, "robot_description::position_iface_0");
    } catch (std::string e) {
      std::cout << "Gazebo error: Unable to connect to the position interface\n" << e << "\n";
      return;
    }
    
    // Enable the motor
    this->posIface->Lock(1);
    this->posIface->data->cmdEnableMotors = 1;
    this->posIface->Unlock();

    this->rnh_ = new ros::NodeHandle();
    //this->sub_ = rnh_->subscribe<geometry_msgs::Twist>("/cmd_vel", 100, boost::bind(&DiffDrive::cmdVelCallBack,this,_1));
    this->sub_ = rnh_->subscribe<geometry_msgs::Twist>("/cmd_vel", 100, &DiffDrive::cmdVelCallBack,this);
    this->pub_ = rnh_->advertise<nav_msgs::Odometry>("/odom", 1);
   
    // spawn 2 threads by default, ///@todo: make this a parameter
    ros::MultiThreadedSpinner s(2);
    boost::thread spinner_thread( boost::bind( &ros::spin, s ) );

    nav_msgs::Odometry odom;
    tf::TransformBroadcaster tfb;

    ros::Duration d; d.fromSec(0.01);
    
    while(rnh_->ok()) { 
      if (this->posIface) {
        this->posIface->Lock(1);
        
        btQuaternion qt; qt.setEulerZYX(this->posIface->data->pose.yaw, this->posIface->data->pose.pitch, this->posIface->data->pose.roll);
        btVector3 vt(this->posIface->data->pose.pos.x, this->posIface->data->pose.pos.y, this->posIface->data->pose.pos.z);

        tf::Transform latest_tf(qt, vt);

        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = ros::Time::now();
        tf::Stamped<tf::Transform> tmp_tf_stamped(latest_tf.inverse(),
                                                  transform_expiration,
                                                  "base_link", "odom");
        tfb.sendTransform(tmp_tf_stamped);
        

        odom.pose.pose.position.x = this->posIface->data->pose.pos.x;
        odom.pose.pose.position.y = this->posIface->data->pose.pos.y;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->posIface->data->pose.yaw);
        odom.twist.twist.linear.x = this->posIface->data->velocity.pos.x;
        odom.twist.twist.linear.y = this->posIface->data->velocity.pos.y;
        odom.twist.twist.angular.z = this->posIface->data->velocity.yaw;
        
        odom.header.frame_id = "odom"; 
        
        odom.header.stamp = transform_expiration;
        
        this->pub_.publish(odom); 

        this->posIface->Unlock();
      }
      d.sleep();
    }
  }
  ~DiffDrive() {
    delete this->rnh_;
  }
};




int main(int argc, char** argv) {
  ros::init(argc,argv,"ros_diffdrive");

  DiffDrive d;
  return 0;
}


