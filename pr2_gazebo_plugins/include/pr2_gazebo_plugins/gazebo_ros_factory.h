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

#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Model.hh>

#include <ros/ros.h>
#include "pr2_gazebo_plugins/GazeboModel.h"
#include "pr2_gazebo_plugins/SpawnModel.h"
#include "pr2_gazebo_plugins/DeleteModel.h"

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo Dynamic Plugins
/// @{
/** \defgroup GazeboRosFactory

  \brief A sample gazebo dynamic plugin
  
  This is a gazebo controller that does nothing

  Example Usage:
  \verbatim
    <model:physical name="robot_model1">

      <controller:gazebo_ros_factory name="gazebo_ros_factory_controller" plugin="libgazebo_ros_factory.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>1000.0</updateRate>
      </controller:gazebo_ros_factory>

      <xyz>0.0 0.0 0.02</xyz>
      <rpy>0.0 0.0 0.0 </rpy>

      <!-- a box -->
      <body:box name="test_block">
          <massMatrix>true</massMatrix>
          <mass>1000</mass>
          <ixx>100</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>100</iyy>
          <iyz>0</iyz>
          <izz>100</izz>
          <cx>0</cx>
          <cy>0</cy>
          <cz>0</cz>
          <xyz>0 0 0.002</xyz>
          <rpy>0 -0 0</rpy>
          <geom:box name="test_block_collision_geom">
              <xyz>0 0 10</xyz>
              <rpy>0 0 0</rpy>
              <size>20 20 20</size>
              <visual>
                  <xyz>0 0 0</xyz>
                  <rpy>0 -0 0</rpy>
                  <scale>20 20 20</scale>
                  <mesh>unit_box</mesh>
                  <material>Gazebo/GrassFloor</material>
              </visual>
          </geom:box>
      </body:box>

    </model:physical>
  \endverbatim
 
\{
*/

class GazeboRosFactory : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosFactory(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosFactory();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief controller parent, a model?
  private: gazebo::Model* myParent;

  /// \brief Service Call Name
  private: ParamT<std::string> *spawnModelServiceNameP;
  private: std::string spawnModelServiceName;
  private: ParamT<std::string> *deleteModelServiceNameP;
  private: std::string deleteModelServiceName;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  /// \brief ros service
  private: ros::ServiceServer spawnModelService;
  private: ros::ServiceServer deleteModelService;

  /// \brief check to see if string is a URDF XML
  private: bool IsURDF(std::string robot_model);

  /// \brief check to see if string is a Gazebo Model XML
  private: bool IsGazeboModelXML(std::string robot_model);

  /// \brief push xml to factory, returns true if successful
  private: bool pushToFactory(std::string gazebo_model_xml);

  /// \brief push model name to iface queue for deletion
  private: bool pushToDeleteQueue(std::string model_name);

  /// \brief ros service call to spawn model via factory
  private: bool spawnModel(pr2_gazebo_plugins::SpawnModel::Request &req,
                           pr2_gazebo_plugins::SpawnModel::Response &res);

  /// \brief ros service call to delete model in Gazebo
  private: bool deleteModel(pr2_gazebo_plugins::DeleteModel::Request &req,
                            pr2_gazebo_plugins::DeleteModel::Response &res);
};

/** \} */
/// @}

}
#endif

