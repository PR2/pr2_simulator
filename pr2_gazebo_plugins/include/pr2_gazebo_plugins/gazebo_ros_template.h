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
#include <gazebo/World.hh>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo Dynamic Plugins
/// @{
/** \defgroup GazeboRosTemplate

  \brief A sample gazebo dynamic plugin
  
  This is a gazebo controller that does nothing

  Example Usage:
  \verbatim
    <model:physical name="robot_model1">

      <controller:gazebo_ros_template name="gazebo_ros_template_controller" plugin="libgazebo_ros_template.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>1000.0</updateRate>
      </controller:gazebo_ros_template>

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

class GazeboRosTemplate : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosTemplate(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosTemplate();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

};

/** \} */
/// @}

}
#endif

