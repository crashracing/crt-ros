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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <sensor_msgs/BatteryState.h>

#include "hardware_measurements.h"

namespace crt_rviz_plugins
{

// BEGIN_TUTORIAL
// Here is the implementation of the HardwareMeasurements class.  HardwareMeasurements
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
HardwareMeasurements::HardwareMeasurements( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  // Then create the control widget.
  QHBoxLayout* battery_layout = new QHBoxLayout;
  battery_layout->addWidget(new ScaleTextlabel("Battery:"));
  battery_widget_ = new ScaleTextlabel("n/a");
  battery_layout->addWidget(battery_widget_);

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  //layout->addLayout( topic_layout );
  layout->addLayout(battery_layout);
  setLayout( layout );

  battery_sub_ = nh_.subscribe("crtbot/battery", 1, &HardwareMeasurements::BatteryCallback, this);

  // Next we make signal/slot connections.
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void HardwareMeasurements::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

// Set the topic name we are publishing to.
void HardwareMeasurements::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
      battery_sub_.shutdown();
    }
    else
    {
      // The old ``velocity_publisher_`` is destroyed by this assignment,
      // and thus the old topic advertisement is removed.  The call to
      // nh_advertise() says we want to publish data on the new topic
      // name.
      battery_sub_ = nh_.subscribe(output_topic_.toStdString(), 1, &HardwareMeasurements::BatteryCallback, this);
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }
}

void HardwareMeasurements::BatteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    const float batv = msg->voltage;
    battery_widget_->setText(QString("%1 V").arg(batv));

    if (batv > 15.2) {
        battery_widget_->setStyleSheet("QLabel { color: lime; background-color: black; font-weight: bold; }");
    } else if (batv > 15.0) {
        battery_widget_->setStyleSheet("QLabel { color: yellow; background-color: black; font-weight: bold; }");
    } else {
        battery_widget_->setStyleSheet("QLabel { color: red; background-color: black; font-weight: bold; }");
    }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void HardwareMeasurements::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void HardwareMeasurements::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace crt_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(crt_rviz_plugins::HardwareMeasurements,rviz::Panel )
// END_TUTORIAL
