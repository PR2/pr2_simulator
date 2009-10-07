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
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef ROS_IMU_HH
#define ROS_IMU_HH

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>
#include <gazebo/Param.hh>

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <sensor_msgs/Imu.h>

namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
   /** \defgroup RosIMU Groud Truth Position Pose and Rates Interface

   \brief RosIMU controller.

   This controller requires to a model as its parent. The plugin broadcasts a body's pose and rates through ROS sensor_msgs::Imu message.  In the example below, the plubin broadcasts pose and rate of a body named \b body_name over ROS topic name \b body_pose_groud_truth.

   Example Usage:
   \verbatim
     <model:physical name="some_fancy_model">
       <controller:ros_imu name="imu_controller" plugin="libros_imu.so">
         <alwaysOn>true</alwaysOn>
         <updateRate>1000.0</updateRate>
         <bodyName>body_name</bodyName>
         <topicName>body_pose_ground_truth</topicName>
         <frameName>map</frameName>
         <xyzOffsets>25.65 25.65 0</xyzOffsets> <!-- option to initialize odometry for fake localization-->
         <rpyOffsets>0 0 0</rpyOffsets>
         <interface:position name="imu_position_iface"/>
       </controller:ros_imu>
     </model:phyiscal>
   \endverbatim
   
\{
*/

/**

   \brief RosIMU controller
          \li Starts a ROS node if none exists.
          \li This controller simulates a 6 dof position and rate sensor, publishes sensor_msgs::Imu.msg ROS topic.
          \li Example Usage:
   \verbatim
     <model:physical name="some_fancy_model">
       <controller:ros_imu name="imu_controller" plugin="libros_imu.so">
         <alwaysOn>true</alwaysOn>
         <updateRate>1000.0</updateRate>
         <bodyName>body_name</bodyName>
         <topicName>body_pose_ground_truth</topicName>
         <frameName>map</frameName>
         <xyzOffsets>25.65 25.65 0</xyzOffsets> <!-- option to initialize odometry for fake localization-->
         <rpyOffsets>0 0 0</rpyOffsets>
         <interface:position name="imu_position_iface"/>
       </controller:ros_imu>
     </model:phyiscal>
   \endverbatim
          .
*/

   class RosIMU : public Controller
   {
      /// \brief Constructor
      public: RosIMU(Entity *parent );

      /// \brief Destructor
      public: virtual ~RosIMU();

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

      /// \brief The parent Model
      private: Body *myBody; //Gazebo/ODE body



      /// \brief pointer to ros node
      private: ros::NodeHandle* rosnode_;
      private: ros::Publisher pub_;

      /// \brief ros message
      private: sensor_msgs::Imu imuMsg;

      /// \brief store bodyname
      private: ParamT<std::string> *bodyNameP;
      private: std::string bodyName;

      /// \brief topic name
      private: ParamT<std::string> *topicNameP;
      private: std::string topicName;

      /// \brief frame transform name, should match link name
      /// FIXME: extract link name directly?
      private: ParamT<std::string> *frameNameP;
      private: std::string frameName;

      /// \brief allow specifying constant xyz and rpy offsets
      private: ParamT<Vector3> *xyzOffsetsP;
      private: Vector3 xyzOffsets;
      private: ParamT<Vector3> *rpyOffsetsP;
      private: Vector3 rpyOffsets;

      /// \brief A mutex to lock access to fields that are used in message callbacks
      private: boost::mutex lock;

      /// \brief save last_time
      private: double last_time;
      private: Vector3 last_vpos;
      private: Vector3 last_veul;
      private: Vector3 apos;
      private: Vector3 aeul;

      /// \brief Gaussian noise
      private: ParamT<double> *gaussianNoiseP;
      private: double gaussianNoise;

      /// \brief Gaussian noise generator
      private: double GaussianKernel(double mu,double sigma);

      /// \brief for setting ROS name space
      private: ParamT<std::string> *robotNamespaceP;
      private: std::string robotNamespace;
   };

/** \} */
/// @}


}

#endif

