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

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
#include <gazebo_plugins/GazeboRosCameraConfig.h>
#include <dynamic_reconfigure/server.h>
#endif

// gazebo stuff
#include "gazebo.h"
#include "sdf/interface/Param.hh"
#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "sensors/SensorTypes.hh"
#include "plugins/CameraPlugin.hh"

namespace gazebo
{

class GazeboRosProsilica : public CameraPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
  public: GazeboRosProsilica();

  /// \brief Destructor
  public: virtual ~GazeboRosProsilica();

  /// \brief Load the controller
  /// \param node XML config node
  public: void Load(sensors::SensorPtr &_parent, sdf::ElementPtr &_sdf);

  /// \brief Init the controller
  protected: virtual void InitChild();

  /// \brief Update the controller
  protected: virtual void UpdateChild();

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
  //private: MonoCameraSensor *myParent;
    // Pointer to the model
    private: physics::WorldPtr world;
    /// \brief The parent sensor
    private: sensors::SensorPtr parentSensor;
    private: sensors::CameraSensorPtr parentCameraSensor;


  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;

  /// \brief image_transport
  private: polled_camera::PublicationServer poll_srv_;      // Handles requests in polled mode

  private: std::string mode_;

  private: image_transport::ImageTransport* itnode_;
  private: image_transport::Publisher image_pub_;
  private: ros::Publisher camera_info_pub_;

/*
  /// \brief Service call to publish images, cam info
  private: bool camInfoService(prosilica_camera::CameraInfo::Request &req,
                               prosilica_camera::CameraInfo::Response &res);
  private: bool triggeredGrab(prosilica_camera::PolledImage::Request &req,
                              prosilica_camera::PolledImage::Response &res);
*/

  private: void pollCallback(polled_camera::GetPolledImage::Request& req,
                             polled_camera::GetPolledImage::Response& rsp,
                             sensor_msgs::Image& image, sensor_msgs::CameraInfo& info);

  /// \brief ros message
  /// \brief construct raw stereo message
  private: sensor_msgs::Image imageMsg;
  private: sensor_msgs::Image *roiImageMsg;
  private: sensor_msgs::CameraInfo cameraInfoMsg;
  private: sensor_msgs::CameraInfo *roiCameraInfoMsg;

  /// \brief for setting ROS name space
  // private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief ROS camera name
  private: std::string cameraName;

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

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
  // Allow dynamic reconfiguration of camera params
  dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig> *dyn_srv_;

  void configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level);

  // Name of camera
  std::string dynamicReconfigureName;
#endif

#ifdef USE_CBQ
  private: ros::CallbackQueue prosilica_queue_;
  private: void ProsilicaQueueThread();
  private: boost::thread prosilica_callback_queue_thread_;
#else
  private: void ProsilicaROSThread();
  private: boost::thread ros_spinner_thread_;
#endif

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    //private: transport::NodePtr node;
    //private: transport::SubscriberPtr statsSub;
    //private: common::Time simTime;
    //public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg);

};

}
#endif

