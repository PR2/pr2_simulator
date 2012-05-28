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

#ifndef LCDRANGE_H
#define LCDRANGE_H

#include <ros/ros.h>
#include <QLCDNumber>
#include <QWidget>
#include <geometry_msgs/Twist.h>

// for torso
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

// for gripper
#include <pr2_controllers_msgs/Pr2GripperCommand.h>

class QLabel;
class QSlider;

class QtRosSlider : public QWidget
{
    Q_OBJECT

public:
    QtRosSlider(const QString &text, QWidget *_parent = 0);

    int value() const;
    QString text() const;

    void setCallback(actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> *_ac);
    void addGripperPublisher(ros::Publisher _pub);

    int min_int;
    int max_int;
    double min_value;
    double max_value;
public slots:
    void setValue(double value);
    void setRange(int minValue, int maxValue);
    void setText(const QString &text);
    void conversion(int value);
    double convert(int value);
    void onValueChange(int value);

signals:
    void valueChanged(int newValue);

private:
    void initTorsoCommander();
    void initGripperCommander();
    QLCDNumber *lcd;
    QSlider *slider;
    QLabel *label;
    actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> *ac;
    std::vector<ros::Publisher> gripper_publishers;
    std::vector<ros::Publisher> base_publishers;
};

#endif
