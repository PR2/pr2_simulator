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
 * Desc: A dynamic controller plugin that performs generic force interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id$
 */
#ifndef ROS_FORCE_HH
#define ROS_FORCE_HH

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <gazebo/Controller.hh>
#include <gazebo/Param.hh>
#include <gazebo/Body.hh>
#include <gazebo/Model.hh>
#include <geometry_msgs/Wrench.h>

namespace gazebo
{

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosForce Plugin XML Reference and Example

  \brief Ros Force Controller.
  
  This is a controller that collects data from a ROS topic and positions a body accordingly.

  Example Usage:
  \verbatim
  <model:physical name="box_model">
    <body:empty name="box_body">
     ...
    </body:empty>
    <controller:gazebo_ros_force name="box_force_controller" plugin="libgazebo_ros_force.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>15.0</updateRate>
        <topicName>box_force</topicName>
        <bodyName>box_body</bodyName>
    </controller:gazebo_ros_force>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/**
           .
 
*/

class GazeboRosForce : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosForce(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosForce();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller, unadvertise topics
  protected: virtual void FiniChild();

  /// \brief call back when a Wrench message is published
  private: void UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& wrenchMsg);

  /// \brief A pointer to the parent entity
  private: Model *myParent;

  /// \brief A pointer to the Body
  private: Body *myBody;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock;

  /// \brief ROS Wrench topic name
  /// \brief inputs
  private: ParamT<std::string> *topicNameP,*bodyNameP;
  private: std::string topicName,bodyName;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;
};

/** \} */
/// @}

}
#endif

