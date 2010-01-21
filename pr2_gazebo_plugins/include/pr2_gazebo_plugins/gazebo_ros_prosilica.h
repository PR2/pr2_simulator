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
 * Desc: A dynamic controller plugin that publishes ROS image topic for generic camera sensor.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * SVN: $Id$
 */
#ifndef ROS_CAMERA_HH
#define ROS_CAMERA_HH

#include <ros/ros.h>
#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#endif
#include "boost/thread/mutex.hpp"
#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>

// image components
#include "cv_bridge/CvBridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
// used by polled_camera
#include "sensor_msgs/RegionOfInterest.h"

// prosilica components
// Stuff in image_common
#include <image_transport/image_transport.h>
#include <polled_camera/publication_server.h>
#include <polled_camera/GetPolledImage.h>



namespace gazebo
{
  class MonoCameraSensor;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosProsilica Ros Camera Plugin XML Reference and Example

  \brief Ros Camera Plugin Controller.
  
  This is a controller that collects data from a Camera Sensor and populates a libgazebo camera interface as well as publish a ROS sensor_msgs::Image (under the field \b \<topicName\>). This controller should only be used as a child of a camera sensor (see example below.

  Example Usage:
  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body_name">
      <sensor:camera name="camera_sensor">
        <controller:gazebo_ros_prosilica name="controller-name" plugin="libgazebo_ros_prosilica.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>camera_name/image</topicName>
            <frameName>camera_body_name</frameName>
        </controller:gazebo_ros_prosilica>
      </sensor:camera>
    </body:empty>
  </model:physical>
  \endverbatim
 
\{
*/

/**


    \brief GazeboRosProsilica Controller.
           \li Starts a ROS node if none exists. \n
           \li Simulates a generic camera and broadcast sensor_msgs::Image topic over ROS.
           \li Example Usage:
  \verbatim
  <model:physical name="camera_model">
    <body:empty name="camera_body_name">
      <sensor:camera name="camera_sensor">
        <controller:gazebo_ros_prosilica name="controller-name" plugin="libgazebo_ros_prosilica.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>15.0</updateRate>
            <topicName>camera_name/image</topicName>
            <frameName>camera_body_name</frameName>
        </controller:gazebo_ros_prosilica>
      </sensor:camera>
    </body:empty>
  </model:phyiscal>
  \endverbatim
           .
 
*/

class GazeboRosProsilica : public Controller
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosProsilica(Entity *parent);

  /// \brief Destructor
  public: virtual ~GazeboRosProsilica();

  /// \brief Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

  /// \brief Finalize the controller, unadvertise topics
  protected: virtual void FiniChild();

  /// \brief does nothing for now
  private: static void mouse_cb(int event, int x, int y, int flags, void* param) { };

  /// \brief Keep track of number of connctions
  private: int imageConnectCount;
  private: void ImageConnect();
  private: void ImageDisconnect();
  private: int infoConnectCount;
  private: void InfoConnect();
  private: void InfoDisconnect();

  private: void PutCameraData();
  private: void PublishCameraInfo();

  /// \brief A pointer to the parent camera sensor
  private: MonoCameraSensor *myParent;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;

  /// \brief image_transport
  private: polled_camera::PublicationServer poll_srv_;      // Handles requests in polled mode

  private: std::string mode_;

  private: ros::Publisher image_pub_;
  private: ros::Publisher camera_info_pub_;

/*
  /// \brief Service call to publish images, cam info
  private: bool camInfoService(prosilica_camera::CameraInfo::Request &req,
                               prosilica_camera::CameraInfo::Response &res);
  private: bool triggeredGrab(prosilica_camera::PolledImage::Request &req,
                              prosilica_camera::PolledImage::Response &res);
*/

  private: bool pollCallback(polled_camera::GetPolledImage::Request& req,
                            sensor_msgs::Image& image, sensor_msgs::CameraInfo& info);

  /// \brief ros message
  /// \brief construct raw stereo message
  private: sensor_msgs::Image imageMsg;
  private: sensor_msgs::Image *roiImageMsg;
  private: sensor_msgs::CameraInfo cameraInfoMsg;
  private: sensor_msgs::CameraInfo *roiCameraInfoMsg;


  /// \brief Parameters
  private: ParamT<std::string> *imageTopicNameP;
  private: ParamT<std::string> *cameraInfoTopicNameP;
  private: ParamT<std::string> *pollServiceNameP;
  private: ParamT<std::string> *frameNameP;

  private: ParamT<double> *CxPrimeP;           // rectified optical center x, for sim, CxPrime == Cx
  private: ParamT<double> *CxP;            // optical center x
  private: ParamT<double> *CyP;            // optical center y
  private: ParamT<double> *focal_lengthP;  // also known as focal length
  private: ParamT<double> *hack_baselineP;  // also known as focal length
  private: ParamT<double> *distortion_k1P; // linear distortion
  private: ParamT<double> *distortion_k2P; // quadratic distortion
  private: ParamT<double> *distortion_k3P; // cubic distortion
  private: ParamT<double> *distortion_t1P; // tangential distortion
  private: ParamT<double> *distortion_t2P; // tangential distortion

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief ROS image topic name
  private: std::string imageTopicName;
  private: std::string cameraInfoTopicName;
  private: std::string pollServiceName;
  private: double CxPrime;
  private: double Cx;
  private: double Cy;
  private: double focal_length;
  private: double hack_baseline;
  private: double distortion_k1;
  private: double distortion_k2;
  private: double distortion_k3;
  private: double distortion_t1;
  private: double distortion_t2;

  /// \brief ROS frame transform name to use in the image message header.
  ///        This should typically match the link name the sensor is attached.
  private: std::string frameName;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock;

  /// \brief size of image buffer
  private: int height, width, depth;
  private: std::string type;
  private: int skip;

#ifdef USE_CBQ
  private: ros::CallbackQueue prosilica_queue_;
  private: void ProsilicaQueueThread();
  private: boost::thread* prosilica_callback_queue_thread_;
#else
  private: boost::thread* ros_spinner_thread_;
#endif

};

/** \} */
/// @}

}
#endif

