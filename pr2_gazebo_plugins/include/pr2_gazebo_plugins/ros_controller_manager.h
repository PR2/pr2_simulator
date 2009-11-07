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

#ifndef GAZEBO_CONTROLLER_MANAGER_H
#define GAZEBO_CONTROLLER_MANAGER_H

#include <vector>
#include <map>
#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include "pr2_hardware_interface/hardware_interface.h"
#include "pr2_controller_manager/controller_manager.h"
#include "pr2_gazebo_plugins/SetModelsJointsStates.h"
#include "pr2_mechanism_model/robot.h"
#include "tinyxml/tinyxml.h"
#include <gazebo/Param.hh>

namespace gazebo
{
class HingeJoint;
class XMLConfigNode;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup ros_controller_manager GazeboMechanismControl class

  \brief GazeboMechanismControl Plugin
  
  This is a controller that provides interface between simulator and the Robot Mechanism Control.
  GazeboMechanismControl requires model as its parent.

  Example Usage:
  \verbatim
  <model:physical name="ray_model">
    <!-- GazeboMechanismControl -->
    <controller:ros_controller_manager name="ros_controller_manager" plugin="libros_controller_manager.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1000.0</updateRate>
      <robot filename="pr2.xml" /> <!-- ros_controller_manager use this file to extract mechanism model -->
      <gazebo_physics filename="gazebo_joints.xml" /> <!-- for simulator/physics specific settigs, currently just damping -->
    </controller:ros_controller_manager>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/**


   Gazebo simulator provides joint level control for mechanisms.  In order to work with mechanisms in real life
   at the level of actuators, a plugin is required.
   As implemented here in GazeboMechanismControl, this plugin abstracts the definitions of
   actuators and transmissions.  It parses the \e robot.xml, \e actuators.xml
   and \e transmissions.xml, then sets up an abstract layer of actuators.  The entire chain of command from
   controllers to actuators to simulated mechanism joints and back are implemented in this plugin.
  
   - On the software/controller side:
     -# The plugin maintians a list of \c fake-actuators as described by \e actuators.xml, from which
        the actuator's \b encoder-value is transmitted to \b joint-state via \e transmissions.xml
     -# The controller reads \b joint-state from \c Mechanism-State and sends \b joint-error-value
        to the PID controller, then issues the resulting \b joint-torque-command to \c Mechanism-Model
     -# \b joint-torque-command is converted to \b actuator-current-command
        via transmission definition from \e transmissions.xml
   - On the Hardware side in the simulator
     -# The plugin maintains a list of \c fake-actuators as described by \e actuators.xml,
        from which the simulator reads the \b actuator-current-command, reverse maps to \b joint-torque-command
        and stores in a set of \c fake-mechanism-states
     -# The \b Joint-torque-command is sent to simulated joint in ODE
     -# \b Simulator-joint-state is obtained from ODE and stored in \c fake-mechanism-states.
     -# \c Fake-mechanism-state's \b joint-state is converted to
        \b actuator-encoder values and stored in \c fake-actuators as defined by \e transmissions.xml
   - On the software/controller side:
     -# [loops around] \b Actuator-encoder-value is transmitted to \b joint-state via \e transmissions.xml
     -# Controller reads \b joint-state and issues a \b joint-torque-command
   .
  
   @image html "http://pr.willowgarage.com/wiki/pr2_gazebo_plugins?action=AttachFile&do=get&target=gazebo_mcn.jpg" "Gazebo Mechanism Control Model"
  
   - Example Usage:
  \verbatim
  <model:physical name="ray_model">
    <!-- GazeboMechanismControl -->
    <controller:ros_controller_manager name="ros_controller_manager" plugin="libros_controller_manager.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1000.0</updateRate>
      <robot filename="pr2.xml" /> <!-- ros_controller_manager use this file to extract mechanism model -->
      <gazebo_physics filename="gazebo_joints.xml" /> <!-- for simulator/physics specific settigs, currently just damping -->
    </controller:ros_controller_manager>
  </model:phyiscal>
  \endverbatim
   .
**/


class RosControllerManager : public gazebo::Controller
{
public:
  RosControllerManager(Entity *parent);
  virtual ~RosControllerManager();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  Model *parent_model_;
  pr2_hardware_interface::HardwareInterface hw_;
  pr2_controller_manager::ControllerManager *cm_;

  /// @todo The fake state helps Gazebo run the transmissions backwards, so
  ///       that it can figure out what its joints should do based on the
  ///       actuator values.
  pr2_mechanism_model::RobotState *fake_state_;
  std::vector<gazebo::Joint*>  joints_;

  /// \brief Service Call Name
  private: ParamT<std::string> *setModelsJointsStatesServiceNameP;
  private: std::string setModelsJointsStatesServiceName;

  /*
   * \brief read pr2.xml for actuators, and pass tinyxml node to mechanism control node's initXml.
   */
  void ReadPr2Xml(XMLConfigNode *node);

  /*
   *  \brief pointer to ros node
   */
  ros::NodeHandle* rosnode_;

  /// \brief ros service
  private: ros::ServiceServer setModelsJointsStatesService;

  ///\brief ros service callback
  private: bool setModelsJointsStates(pr2_gazebo_plugins::SetModelsJointsStates::Request &req,
                                      pr2_gazebo_plugins::SetModelsJointsStates::Response &res);

  ///\brief ros service callback
  /*
   *  \brief tmp vars for performance checking
   */
  double wall_start, sim_start;

  /// \brief set topic name of robot description parameter
  ParamT<std::string> *robotParamP;
  ParamT<std::string> *robotNamespaceP;
  std::string robotParam;
  std::string robotNamespace;

};

/** \} */
/// @}

}

#endif

