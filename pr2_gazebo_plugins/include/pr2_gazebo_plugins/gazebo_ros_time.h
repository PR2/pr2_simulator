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
#include <gazebo/Param.hh>

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"

// roscpp - used for broadcasting time over ros
#include <roslib/Time.h>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosTime ROS time broadcaster.

  \brief Broadcast simulator time over ROS
  
  This is a controller that broadcasts simulator time over ros::time

  Example Usage:
  \verbatim
    <model:physical name="robot_model1">

      <controller:gazebo_ros_time name="gazebo_ros_time" plugin="libgazebo_ros_time.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>1000.0</updateRate>
      </controller:gazebo_ros_time>

      <xyz>0.0 0.0 0.02</xyz>
      <rpy>0.0 0.0 0.0 </rpy>

      <!-- base, torso and arms -->
      <include embedded="true">
        <xi:include href="pr2_xml.model" />
      </include>

    </model:physical>
  \endverbatim
 
\{
*/

/**
   
    \brief ROS Time Controller
      \li Starts a ROS node if none exists
      \li broadcast simulator time over roslib::Time.
      \li Example Usage:
          \verbatim
            <model:physical name="robot_model1">

              <controller:gazebo_ros_time name="gazebo_ros_time" plugin="libgazebo_ros_time.so">
                <alwaysOn>true</alwaysOn>
                <updateRate>1000.0</updateRate>
              </controller:gazebo_ros_time>

              <xyz>0.0 0.0 0.02</xyz>
              <rpy>0.0 0.0 0.0 </rpy>

              <!-- base, torso and arms -->
              <include embedded="true">
                <xi:include href="pr2_xml.model" />
              </include>

            </model:physical>
          \endverbatim
      .

**/
class GazeboRosTime : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosTime(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosTime();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  roslib::Time timeMsg;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

};

/** \} */
/// @}

}
#endif

