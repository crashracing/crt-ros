/*********************************************************************
 * Original Code: Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the CRTBot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef CRTBOT_CONTROL__CRTBOT_HW_INTERFACE_H
#define CRTBOT_CONTROL__CRTBOT_HW_INTERFACE_H

#include <string>
#include <vector>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>

namespace crtbot_control
{

static const std::string RESET_MAGIC = "LMC_RESET_2342";

class RobustSerial
{
public:
    RobustSerial() = default;

    RobustSerial(const std::string port, const unsigned baud,
                 const unsigned timeout=1000) :
        port(port),
        baud(baud),
        timeout(timeout) {
    }

    bool tryConnect(bool blocking=false)
    {
        if (serial && serial->isOpen()) {
            // already/still open
            return true;
        }

        bool done = false;
        bool success = false;
        while (!done) {
            if (!blocking) {
                done = true;
            }

            if (!serial) {
                // try to open port
                try {
                    serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(timeout));
                } catch (const serial::IOException &e) {
                    ROS_ERROR_STREAM_THROTTLE(10, "serial create on port " << port << " failed: " << e.what());
                    serial = nullptr;
                    // try again
                    continue;
                }

                // it exists now
            }

            if (serial) {
                if (!serial->isOpen()) {
                    // not open, kill it to create it anew
                    delete serial;
                    serial = nullptr;
                } else {
                    success = true;
                    done = true;
                }
            }
        }

        return success;
    }

    bool write(std::string buf) {
        if (!tryConnect()) {
            return false;
        }

        try {
            serial->write(buf);
        } catch (const serial::SerialException &e) {
            ROS_ERROR_STREAM("serial write on port " << port << " failed: " << e.what());
            serial->close();
            return false;
        } catch (const serial::IOException &e) {
            ROS_ERROR_STREAM("serial write on port " << port << " failed: " << e.what());
            serial->close();
            return false;
        }

        return true;
    }

    serial::Serial *serial = nullptr;

protected:
    std::string port;
    unsigned baud;
    unsigned timeout;
};

/// \brief Hardware interface for a robot
class CRTBotHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  CRTBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

protected:
  void tryConnectRelay();
  bool tryConnect(bool blocking);
  void dataInput(const std::string data, unsigned tina_package_depth=0);
  void handlePacketShell(std::string line);
  void handlePacketTina(std::string line);

  void PrepareCb(const std_msgs::EmptyConstPtr&);
  void CleanWheelsCb(const std_msgs::EmptyConstPtr&);

  RobustSerial lmc_serial;
  RobustSerial ds_serial;

  static constexpr bool PACKET_DEBUG = false;
  bool got_response = false;
  std::vector<std::string> packageBuffer_;

  ros::Publisher battery_pub;
  ros::Subscriber prepare_sub;
  ros::Subscriber cleanwheels_sub;
  tf::TransformBroadcaster tf_pub;

  bool request_prepare = false;
  bool request_cleanwheels = false;
  bool last_moving = false;
};  // class

}  // namespace

#endif
