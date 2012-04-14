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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

#include <boost/function.hpp>
#include <boost/thread.hpp>

/*#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/World.hh>
*/

#include <QApplication>
#include <QFont>
#include <QLCDNumber>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>
#include "pr2_gazebo/qt_ros_slider.h"

class MyWidget : public QWidget
{
  public:
    MyWidget(QWidget *parent, ros::NodeHandle* rosnode)
        : QWidget(parent)
    {
        if (!ros::isInitialized()) {
          ROS_ERROR("ros node didn't start for some reason");
          return;
        }
        actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> *ac;

        QVBoxLayout *layout = new QVBoxLayout;
        QPushButton *quit = new QPushButton(tr("Quit"));
        quit->setFont(QFont("Times", 18, QFont::Bold));
        connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
        layout->addWidget(quit);

        // /base_controller/command
        // /head_traj_controller/command
        // /l_arm_controller/command
        // /l_gripper_controller/command
        // /r_arm_controller/command
        // /r_gripper_controller/command
        // /torso_controller/command

        {
          std::string ac_name("/torso_controller/position_joint_action");
          ac = new actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction>(ac_name,true);
          while (!ac->waitForServer(ros::Duration(5.0))) ROS_INFO("waiting for torso action");

          QtRosSlider *lcd = new QtRosSlider("torso height",parent);
          lcd->setCallback(ac);
          lcd->min_value = 0.0;
          lcd->max_value = 0.31;
          lcd->setValue(0.0);
          layout->addWidget(lcd);
        }
        {
          std::string pub_name("/l_gripper_controller/command");
          QtRosSlider *lcd = new QtRosSlider("left gripper",parent);
          ros::Publisher pub = rosnode->advertise<pr2_controllers_msgs::Pr2GripperCommand>(pub_name,1);
          lcd->addGripperPublisher(pub);
          lcd->min_value = 0.0;
          lcd->max_value = 0.08;
          lcd->setValue(0.0);
          layout->addWidget(lcd);
        }
        {
          std::string pub_name("/r_gripper_controller/command");
          QtRosSlider *lcd = new QtRosSlider("right gripper",parent);
          ros::Publisher pub = rosnode->advertise<pr2_controllers_msgs::Pr2GripperCommand>(pub_name,1);
          lcd->addGripperPublisher(pub);
          lcd->min_value = 0.0;
          lcd->max_value = 0.08;
          lcd->setValue(0.0);
          layout->addWidget(lcd);
        }

        setLayout(layout);
    }
};

void QTThread(ros::NodeHandle* rosnode)
{
  // start up qt
  int argc=0;
  char** argv = NULL;
  QApplication app(argc, argv);
  MyWidget widget(NULL,rosnode);
  widget.show();
  app.exec();
  rosnode->shutdown();
}

void QueueThread(ros::NodeHandle* rosnode)
{
  static const double timeout = 0.01;
  ros::CallbackQueue srp_queue_;
  while (rosnode->ok())
  {
    srp_queue_.callAvailable(ros::WallDuration(timeout));
  }
  srp_queue_.clear();
  srp_queue_.disable();
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"controller_sliders");
  }

  ros::NodeHandle* rosnode_ = new ros::NodeHandle();



  boost::thread qt_thread_ = boost::thread( boost::bind( &QTThread,_1), rosnode_ );
  boost::thread callback_queue_thread_ = boost::thread( boost::bind( &QueueThread, _1 ), rosnode_ );


  callback_queue_thread_.join();
  qt_thread_.join();
}


