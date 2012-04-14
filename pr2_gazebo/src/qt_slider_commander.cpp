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
#include "pr2_gazebo/lcdrange.h"


class MyWidget : public QWidget
{
  public:
    MyWidget(QWidget *parent, ros::NodeHandle* rosnode)
        : QWidget(parent)
    {
        QVBoxLayout *layout = new QVBoxLayout;

        QPushButton *quit = new QPushButton(tr("Quit"));
        quit->setFont(QFont("Times", 18, QFont::Bold));
        connect(quit, SIGNAL(clicked()), qApp, SLOT(quit()));
        layout->addWidget(quit);


        std::vector<std::string> topic_names;
        topic_names.push_back("/trunk_controller/command");
        topic_names.push_back("/shoulder_controller/command");
        topic_names.push_back("/upper_arm_controller/command");
        topic_names.push_back("/forearm_base_controller/command");
        topic_names.push_back("/forearm_controller/command");
        topic_names.push_back("/wrist_diff_controller/command");
        topic_names.push_back("/wrist_controller/command");

        std::vector<ros::Publisher> publishers;
        for (unsigned int i=0 ; i< topic_names.size(); ++i)
          publishers.push_back(rosnode->advertise<std_msgs::Float64>(topic_names[i],1));

        for (unsigned int i=0 ; i< topic_names.size(); ++i)
        {
          std::vector<ros::Publisher> publishers_for_this_slide;
          publishers_for_this_slide.push_back(publishers[i]);
          LCDRange *lcd = new LCDRange(parent, publishers_for_this_slide);
          lcd->setValue(500);
          layout->addWidget(lcd);
        }

        {
          std::vector<ros::Publisher> publishers_for_this_slide;
          topic_names.push_back("/l_wheel_controller/command");
          publishers.push_back(rosnode->advertise<std_msgs::Float64>("/l_wheel_controller/command",1));
          publishers_for_this_slide.push_back(publishers[publishers.size()-1]);
          topic_names.push_back("/r_wheel_controller/command");
          publishers.push_back(rosnode->advertise<std_msgs::Float64>("/r_wheel_controller/command",1));
          publishers_for_this_slide.push_back(publishers[publishers.size()-1]);
          LCDRange *lcd = new LCDRange(parent, publishers_for_this_slide);
          lcd->setValue(500);
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


