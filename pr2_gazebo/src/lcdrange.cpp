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

#include "pr2_gazebo/lcdrange.h"

LCDRange::LCDRange(QWidget *parent,std::vector<ros::Publisher> publishers)
    : QWidget(parent)
{
    this->publishers_ = publishers;
    init();
}

LCDRange::LCDRange(const QString &text, QWidget *parent)
    : QWidget(parent)
{
    init();
    setText(text);
}

void LCDRange::init()
{
    this->lcd = new QLCDNumber(5);
    this->lcd->setSegmentStyle(QLCDNumber::Filled);

    this->slider = new QSlider(Qt::Horizontal);
    this->slider->setRange(0, 1000);
    this->slider->setValue(0);
    this->label = new QLabel;
    this->label->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    this->label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

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
void LCDRange::onValueChange(int value)
{
  std_msgs::Float64 data;
  data.data = this->convert(value);
  for (std::vector<ros::Publisher>::iterator pub_iter = this->publishers_.begin();
       pub_iter != this->publishers_.end(); ++pub_iter)
    pub_iter->publish(data);
}

int LCDRange::value() const
{
    return this->slider->value();
}

double LCDRange::convert(int value)
{
  return ((double)value - 500.0)/1000.0 * (4.0*M_PI);
}

void LCDRange::conversion(int value)
{
    this->lcd->display(this->convert(value));
}

QString LCDRange::text() const
{
    return this->label->text();
}

void LCDRange::setValue(int value)
{
    this->slider->setValue(value);
}

void LCDRange::setRange(int minValue, int maxValue)
{
    if (minValue < 0 || maxValue > 999 || minValue > maxValue) {
        qWarning("LCDRange::setRange(%d, %d)\n"
                 "\tRange must be 0..999\n"
                 "\tand minValue must not be greater than maxValue",
                 minValue, maxValue);
        return;
    }
    this->slider->setRange(minValue, maxValue);
}

void LCDRange::setText(const QString &text)
{
    this->label->setText(text);
}

