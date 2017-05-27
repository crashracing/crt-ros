/*
 * Original Code:
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *
 * New Code and Changes:
 *  Copyright (C) 2017  Crash Racing Team <crt@turag.de>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef HARDWARE_MEASUREMENTS_H
#define HARDWARE_MEASUREMENTS_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
# include <sensor_msgs/BatteryState.h>
#endif

#include <QLabel>
#include <QTimer>

class QLineEdit;

namespace crt_rviz_plugins
{

class DriveWidget;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// HardwareMeasurements will show a text-entry field to set the output topic
// and a 2D control area.  The 2D control area is implemented by the
// DriveWidget class, and is described there.
class HardwareMeasurements: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  HardwareMeasurements( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // The control area, DriveWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.
  void setVel( float linear_velocity_, float angular_velocity_ );

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic( const QString& topic );

  // Here we declare some internal slots.
protected Q_SLOTS:

  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateTopic();

  // Then we finish up with protected member variables.
protected:
  void BatteryCallback(const sensor_msgs::BatteryState::ConstPtr&);

  // The control-area widget which turns mouse events into command
  // velocities.
  QLabel *battery_widget_;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* output_topic_editor_;

  // The current name of the output topic.
  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Subscriber battery_sub_;

  // The ROS node handle.
  ros::NodeHandle nh_;
  // END_TUTORIAL
};

// ScaleTextlabel source: https://forum.qt.io/topic/36088/automatically-scale-text-in-qlabels/10
class ScaleTextlabel : public QLabel{
    Q_OBJECT
    Q_DISABLE_COPY(ScaleTextlabel)
public:
    explicit ScaleTextlabel(QWidget* parent=Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags())
        :QLabel(parent,f)
    {
       initTimer();
    }
    explicit ScaleTextlabel(const QString &text, QWidget *parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags())
        :QLabel(text,parent,f)
    {
       initTimer();
    }

protected:
    void resizeEvent(QResizeEvent* event) override{
        QLabel::resizeEvent(event);
        m_resizeTimer->start();
    }
protected Q_SLOTS:
    void resizeFont(){
        QFont basefont = font();
        const QRect baseRect = contentsRect();
        const QString baseText = text();
        int fontSizeGuess = qMax(1,basefont.pixelSize());
        for(;;++fontSizeGuess){
            QFont testFont(basefont);
            testFont.setPixelSize(fontSizeGuess);
            const QRect fontRect=QFontMetrics(testFont).boundingRect(baseText);
            if(fontRect.height()>baseRect.height() || fontRect.width()>baseRect.width())
                break;
        }
        for(;fontSizeGuess>1;--fontSizeGuess){
            QFont testFont(basefont);
            testFont.setPixelSize(fontSizeGuess);
            const QRect fontRect=QFontMetrics(testFont).boundingRect(baseText);
            if(fontRect.height()<=baseRect.height() && fontRect.width()<=baseRect.width())
                break;
        }
        basefont.setPixelSize(fontSizeGuess);
        setFont(basefont);
    }

private:
    void initTimer(){
        m_resizeTimer=new QTimer(this);
        m_resizeTimer->setInterval(100);
        m_resizeTimer->setSingleShot(true);
        connect(m_resizeTimer,&QTimer::timeout,this,&ScaleTextlabel::resizeFont);

        setMinimumSize(0, 16);
        setMaximumSize(1000, 500);
    }

    QTimer* m_resizeTimer;
};

} // end namespace rviz_plugin_tutorials

#endif // HARDWARE_MEASUREMENTS_H
