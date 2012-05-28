/****************************************************************************
**
** Copyright (C) 2005-2007 Trolltech ASA. All rights reserved.
**
** This file is part of the example classes of the Qt Toolkit.
**
** This file may be used under the terms of the GNU General Public
** License version 2.0 as published by the Free Software Foundation
** and appearing in the file LICENSE.GPL included in the packaging of
** this file.  Please review the following information to ensure GNU
** General Public Licensing requirements will be met:
** http://www.trolltech.com/products/qt/opensource.html
**
** If you are unsure which license is appropriate for your use, please
** review the following information:
** http://www.trolltech.com/products/qt/licensing.html or contact the
** sales department at sales@trolltech.com.
**
** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
**
****************************************************************************/
#include <math.h>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>

#include <std_msgs/Float64.h>

#include "pr2_gazebo/qt_ros_slider.h"

QtRosSlider::QtRosSlider(const QString &text, QWidget *_parent)
    : QWidget(_parent)
{
    this->ac = NULL;
    this->gripper_publishers.clear();

    this->min_int = 0;
    this->max_int = 1000;

    this->label = new QLabel();
    this->label->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    this->label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

    setText(text);
}

void QtRosSlider::initTorsoCommander()
{
    this->lcd = new QLCDNumber(5);
    this->lcd->setSegmentStyle(QLCDNumber::Filled);

    this->slider = new QSlider(Qt::Horizontal);
    this->slider->setRange(this->min_int, max_int);

    connect(this->slider, SIGNAL(valueChanged(int)),
            this, SLOT(conversion(int)));

    connect(this->slider, SIGNAL(valueChanged(int)),
            this, SIGNAL(valueChanged(int)));

    connect(this->slider, SIGNAL(valueChanged(int)),
            this, SLOT(onValueChange(int)));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(this->lcd);
    layout->addWidget(this->slider);
    layout->addWidget(this->label);
    setLayout(layout);

    setFocusProxy(this->slider);
}

void QtRosSlider::initGripperCommander()
{
    this->lcd = new QLCDNumber(5);
    this->lcd->setSegmentStyle(QLCDNumber::Filled);

    this->slider = new QSlider(Qt::Horizontal);
    this->slider->setRange(this->min_int, max_int);

    connect(this->slider, SIGNAL(valueChanged(int)),
            this, SLOT(conversion(int)));

    connect(this->slider, SIGNAL(valueChanged(int)),
            this, SIGNAL(valueChanged(int)));

    connect(this->slider, SIGNAL(valueChanged(int)),
            this, SLOT(onValueChange(int)));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(this->lcd);
    layout->addWidget(this->slider);
    layout->addWidget(this->label);
    setLayout(layout);

    setFocusProxy(this->slider);
}

void QtRosSlider::setCallback( actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> *_ac)
{
  this->ac = _ac;
  this->initTorsoCommander();
}

void QtRosSlider::addGripperPublisher(ros::Publisher _pub)
{
  this->gripper_publishers.push_back(_pub);
  this->initGripperCommander();
}

void QtRosSlider::onValueChange(int value)
{
  if (this->ac)
  {
    pr2_controllers_msgs::SingleJointPositionGoal goal;
    goal.position = this->convert(value);
    ac->sendGoal(goal);
  }

  if (!this->gripper_publishers.empty())
  {
    pr2_controllers_msgs::Pr2GripperCommand cmd;
    cmd.position = this->convert(value);
    cmd.max_effort = 100;
    for (std::vector<ros::Publisher>::iterator iter = this->gripper_publishers.begin();
         iter != this->gripper_publishers.end(); iter++)
      iter->publish(cmd);
  }

}

int QtRosSlider::value() const
{
    return this->slider->value();
}

double QtRosSlider::convert(int value)
{
  return ((double)value - min_int)/(max_int - min_int) * (max_value - min_value) + min_value;
}

void QtRosSlider::conversion(int value)
{
    this->lcd->display(this->convert(value));
}

QString QtRosSlider::text() const
{
    return this->label->text();
}

void QtRosSlider::setValue(double value)
{
    int int_value = (int)((value - min_value) / (max_value - min_value) * (max_int - min_int) - min_int);
    this->slider->setValue(int_value);
}

void QtRosSlider::setRange(int minValue, int maxValue)
{
    if (minValue < 0 || maxValue > 999 || minValue > maxValue) {
        qWarning("QtRosSlider::setRange(%d, %d)\n"
                 "\tRange must be 0..999\n"
                 "\tand minValue must not be greater than maxValue",
                 minValue, maxValue);
        return;
    }
    this->slider->setRange(minValue, maxValue);
}

void QtRosSlider::setText(const QString &text)
{
    this->label->setText(text);
}















