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
 * Desc: Actuator array controller for a Pr2 robot.
 * Author: Nathan Koenig
 * Date: 19 Sept 2007
 * SVN: $Id$
 */
#ifndef ROS_TIME_HH
#define ROS_TIME_HH

#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/Model.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <pr2_gazebo_plugins/WorldState.h>

#include <ros/ros.h>

#include <map>
#include <vector>
#include <string>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo Dynamic Plugins
/// @{
/** \defgroup RosStepWorldState

  \brief A sample gazebo dynamic plugin
  
  This is a gazebo controller that does nothing

  Example Usage:
  \verbatim
    <model:physical name="robot_model1">

    </model:physical>
  \endverbatim
 
\{
*/

class RosStepWorldState : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: RosStepWorldState(Entity *parent);

  /// \brief Destructor
  public: virtual ~RosStepWorldState();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief: keep track of number of connections
  private: int worldStateConnectCount;
  private: void WorldStateConnect();
  private: void WorldStateDisconnect();

  /// \brief: Message for sending world state
  private: pr2_gazebo_plugins::WorldState worldStateMsg;

  /// \bridf: Keep list of all models in the world
  private: const std::map<std::string,gazebo::Body*> *bodies;

  /// \bridf: parent should be a model
  private: gazebo::Model* parent_model_;

  /// \bridf: ros node handle and publisher
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;

  /// \brief frame transform name, should match link name
  private: ParamT<std::string> *frameNameP;
  private: std::string frameName;

};

/** \} */
/// @}

}
#endif

