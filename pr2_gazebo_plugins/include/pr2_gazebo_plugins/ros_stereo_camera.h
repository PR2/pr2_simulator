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
 * Desc: A stereo camera controller
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN: $Id:$
 */

#ifndef ROS_STEREO_CAMERA_HH
#define ROS_STEREO_CAMERA_HH

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"

#include <gazebo/Generic_Camera.hh>
#include <gazebo/gazebo.h>
#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>

// raw_stereo components
#include <sensor_msgs/Image.h>
#include "stereo_msgs/RawStereo.h"
#include "stereo_msgs/StereoInfo.h"
#include "sensor_msgs/CameraInfo.h"
#include "stereo_msgs/DisparityInfo.h"

namespace gazebo
{
  class MonoCameraSensor;
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup RosStereoCamera ROS stereo camera controller plugin

  \brief Stereo camera controller.
  
  This is a controller that collects data from a Stereo Camera Sensor and populates a libgazebo stereo camera interfaace. This controller should only be used as a child of a stereo camera sensor 

  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body">
      <sensor:stereocamera name="stereo_camera_sensor">
        <imageSize>640 480</imageSize>
        <hfov>60</hfov>
        <nearClip>0.1</nearClip>
        <farClip>100</farClip>
        <saveFrames>false</saveFrames>
        <saveFramePath>frames</saveFramePath>
        <baseline>0.2</baseline>
        <updateRate>15.0</updateRate>
        <controller:ros_stereocamera name="stereo_camera_controller" plugin="libros_stereo_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>15.0</updateRate>
          <interface:stereocamera name="stereo_iface_0" />
          <interface:camera name="camera_iface_0" />
          <interface:camera name="camera_iface_1" />
          <leftcamera>camera_iface_0</leftcamera>
          <rightcamera>camera_iface_1</rightcamera>
          <leftCloudTopicName>stereo_left_cloud</leftCloudTopicName>
          <rightCloudTopicName>stereo_right_cloud</rightCloudTopicName>
          <leftTopicName>stereo_left_image</leftTopicName>
          <rightTopicName>stereo_right_image</rightTopicName>
          <stereoTopicName>stereo_right_image</stereoTopicName>
          <leftFrameName>stereo_left</leftFrameName>
          <rightFrameName>stereo_right</rightFrameName>
        </controller:ros_stereocamera>
      </sensor:stereocamera>
    </body:empty>
  </model:phyiscal>
  \endverbatim
 
\{
*/

/// \brief Stereo camera controller.
/// 
/// This is a controller that simulates a stereo camera
class RosStereoCamera : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model
  public: RosStereoCamera(Entity *parent);

  /// \brief Destructor
  public: virtual ~RosStereoCamera();

  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Save the controller.
  ///        stream Output stream
  protected: void SaveChild(std::string &prefix, std::ostream &stream);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller
  protected: virtual void FiniChild();

  /// \brief Keep track of number of connctions
  private: int imageConnectCount;
  private: void ImageConnect();
  private: void ImageDisconnect();

  /// \brief The parent sensor
  private: Entity *myParent;

  /// \brief parameters
  private: ParamT<std::string> *leftCameraNameP;
  private: ParamT<std::string> *rightCameraNameP;
  private: ParamT<std::string> *topicNameP;
  private: ParamT<std::string> *frameNameP;
  private: ParamT<double> *CxPrimeP;           // rectified optical center x, for sim, CxPrime == Cx
  private: ParamT<double> *CxP;            // optical center x
  private: ParamT<double> *CyP;            // optical center y
  private: ParamT<double> *focal_lengthP;  // also known as focal length
  private: ParamT<double> *distortion_k1P; // linear distortion
  private: ParamT<double> *distortion_k2P; // quadratic distortion
  private: ParamT<double> *distortion_k3P; // cubic distortion
  private: ParamT<double> *distortion_t1P; // tangential distortion
  private: ParamT<double> *distortion_t2P; // tangential distortion
  private: ParamT<double> *baselineP;      // shift from left camera to right camera.  we treat LEFT camera as origin

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief Pointer to Mono Left and Right Cameras
  private: MonoCameraSensor *leftCamera;
  private: MonoCameraSensor *rightCamera;
  /// \brief Stereo Node stuff
  private: std::string leftCameraName;
  private: std::string rightCameraName;
  private: std::string topicName;
  private: std::string frameName;
  private: double CxPrime;
  private: double Cx;
  private: double Cy;
  private: double focal_length;
  private: double distortion_k1;
  private: double distortion_k2;
  private: double distortion_k3;
  private: double distortion_t1;
  private: double distortion_t2;
  private: double baseline;

  /// \brief pointer to ros node
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;

  /// \brief ros message
  /// \brief construct raw stereo message
  private: stereo_msgs::RawStereo rawStereoMsg;
  private: sensor_msgs::Image* leftImageMsg;
  private: sensor_msgs::Image* rightImageMsg;
  private: sensor_msgs::CameraInfo* leftCameraInfoMsg;
  private: sensor_msgs::CameraInfo* rightCameraInfoMsg;
  private: stereo_msgs::StereoInfo* stereoInfoMsg;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief Put camera data somewhere
  private: void PutCameraData();

  /// \brief The image formats of the cameras 
  private: std::string left_type; 
  private: int left_skip; 
  private: std::string right_type; 
  private: int right_skip; 
};

/** \} */
/// @}

}

#endif

