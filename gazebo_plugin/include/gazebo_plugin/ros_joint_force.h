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
 * Desc: ROS Joint Force Controller
 * Author: John Hsu
 * Date: 24 Sept 2008
 */
#ifndef ROS_JOINT_FORCE_CONTROLLER_HH
#define ROS_JOINT_FORCE_CONTROLLER_HH

/// Maximum number of joints that can be watched by one controller
#define ROS_JOINT_FORCE_CONTROLLER_MAX_FEEDBACKS 16

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <ode/ode.h>
#include <sys/time.h>


namespace gazebo
{
/// \addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// \{
/** \defgroup RosJointforce Joint Force Controller Plugin

  \brief A controller that measures forces and torques exerted by joints

  \verbatim
    <model:physical name="test_model">
      <body:empty name="body_name">
          <controller:ros_joint_force name="ros_ray_sensor_controller" plugin="libros_joint_force.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>jointfoce_topic_name</topicName>
            <frameName>test_model</frameName>
            <interface:opaque name="jointforce_iface" />
          </controller:ros_joint_force>
      </body:empty>
    </model:phyiscal>
  \endverbatim
 
  \{
*/

/// \brief A JointForce controller
class RosJointForce : public Controller
{
  /// \brief Constructor
    public: RosJointForce(Entity *parent );

  /// \brief Destructor
    public: virtual ~RosJointForce();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief The parent Model
  private: Model *myParent;

  /// \brief The joint feedbacks
  private: dJointFeedback *jointfeedbacks[ROS_JOINT_FORCE_CONTROLLER_MAX_FEEDBACKS];
  /// \brief The number of joints we are watching
  private: int n_joints;
};

/** \} */
/// \}

}

#endif

