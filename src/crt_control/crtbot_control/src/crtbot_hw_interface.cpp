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
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <crtbot_control/crtbot_hw_interface.h>
#include <string>
#include <vector>
#include <cstdio>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Empty.h>
#include <serial/serial.h>

namespace crtbot_control
{

CRTBotHWInterface::CRTBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model),
      lmc_serial({"/dev/turag/LMC", 115200}),
      ds_serial({"/tmp/ttycrtros1", 1000000, 10})
{
    ROS_INFO_NAMED("crtbot_hw_interface", "CRTBotHWInterface Ready.");

    packageBuffer_.clear();

    battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 10);

    prepare_sub = nh.subscribe("prepare", 1, &CRTBotHWInterface::PrepareCb, this);
    cleanwheels_sub = nh.subscribe("clean_wheels", 1, &CRTBotHWInterface::CleanWheelsCb, this);

    // setup serial
    lmc_serial.tryConnect();
    ds_serial.tryConnect();
}

void CRTBotHWInterface::read(ros::Duration &elapsed_time)
{
    if (!lmc_serial.tryConnect()) {
        // not connected
        return;
    }

    std::string line;
    try {
        lmc_serial.serial->write("rosgetpose\r\n");
        got_response = false;

        static constexpr unsigned count_limit = 100;
        unsigned count = 0;

        // read data from serial until answer is received and no more bytes to read.
        // stop trying after some ms
        while ((!got_response || lmc_serial.serial->available()) && ++count < count_limit) {
            line = lmc_serial.serial->readline();
            dataInput(line);
        }

        if (count >= count_limit) {
            ROS_ERROR_STREAM("serial read failed: no answer");
        }
    } catch (const serial::SerialException &e) {
        ROS_ERROR_STREAM("serial read failed: " << e.what());
        lmc_serial.serial->close();
        return;
    } catch (const serial::IOException &e) {
        ROS_ERROR_STREAM("serial read failed: " << e.what());
        lmc_serial.serial->close();
        return;
    }
}

void CRTBotHWInterface::write(ros::Duration &elapsed_time)
{
    if (!lmc_serial.tryConnect()) {
        // not connected
        return;
    }

    // Safety
    enforceLimits(elapsed_time);

    //for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    //    joint_position_[joint_id] += joint_position_command_[joint_id];

    const float v_trans_mm_s = 1000.0f * (joint_velocity_command_[1] + joint_velocity_command_[0]) / 2.0f;
    const float v_rot_mrad_s = 1000.0f * (joint_velocity_command_[1] - joint_velocity_command_[0]);
    const bool moving = abs(v_trans_mm_s) > 0.0001 || abs(v_rot_mrad_s) > 0.0001;

    //lmc_serial.serial->flush();
    char buf[256] = { 0 };
    bool send = true;
    if (request_prepare) {
        ROS_INFO("requesting PREPARE");
        snprintf(buf, 256, "rosprepare\r\n");
        request_prepare = false;
    } else if (request_cleanwheels) {
        ROS_INFO("requesting CLEAN_WHEELS");
        snprintf(buf, 256, "rosclean\r\n");
        request_cleanwheels = false;
    } else if (last_moving || moving) {
        snprintf(buf, 256, "roscomm 0 %f %f\r\n", v_trans_mm_s, v_rot_mrad_s);

        // when standing still, don't send further 0's to avoid aborting prepare/clean
        last_moving = moving;
    } else {
        send = false;
    }

    //ROS_INFO_STREAM("TX: cmd (" << joint_velocity_command_[0] << "|" << joint_velocity_command_[1] << ") -> " << buf);
    if (send) {
        if (!lmc_serial.write(buf)) {
            return;
        }
    }

    // relay data to debug server
    if (ds_serial.tryConnect()) {
        if (ds_serial.serial->available()) {
            std::string line = ds_serial.serial->readline();
            if (line.find(RESET_MAGIC) != std::string::npos) {
                lmc_serial.write("\x18\x18\x18\x18\x18\x18\r\nreset_into_bootloader\r\n");
            }
        }
    }
}

void CRTBotHWInterface::enforceLimits(ros::Duration &period)
{
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
    //
    // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
    // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
    // DEPENDING ON YOUR CONTROL METHOD
    //
    // EXAMPLES:
    //
    // Saturation Limits ---------------------------
    //
    // Enforces position and velocity
    //pos_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces velocity and acceleration limits
    vel_jnt_sat_interface_.enforceLimits(period);
    //
    // Enforces position, velocity, and effort
    // eff_jnt_sat_interface_.enforceLimits(period);

    // Soft limits ---------------------------------
    //
    // pos_jnt_soft_limits_.enforceLimits(period);
    // vel_jnt_soft_limits_.enforceLimits(period);
    // eff_jnt_soft_limits_.enforceLimits(period);
    //
    // ----------------------------------------------------
    // ----------------------------------------------------
    // ----------------------------------------------------
}

// source: https://stackoverflow.com/a/1493195
template <class ContainerT>
static void tokenize(const std::string& str, ContainerT& tokens,
              const std::string& delimiters = " ", bool trimEmpty = false)
{
    std::string::size_type pos, lastPos = 0, length = str.length();

    using value_type = typename ContainerT::value_type;
    using size_type  = typename ContainerT::size_type;

    while(lastPos < length + 1)
    {
        pos = str.find_first_of(delimiters, lastPos);
        if(pos == std::string::npos)
        {
            pos = length;
        }

        if(pos != lastPos || !trimEmpty)
            tokens.push_back(value_type(str.data()+lastPos,
                                        (size_type)pos-lastPos ));

        lastPos = pos + 1;
    }
}

static std::string trimmedBuffer(std::string::const_iterator begin, std::string::const_iterator end) {
    for (; *begin == '\r' && begin < end; begin++) { }
    for (; *(end-1) == '\r' && end > begin; end--) { }

    return {begin, end};
}

void CRTBotHWInterface::dataInput(const std::string data, unsigned tina_package_depth) {
    std::string::const_iterator msg_begin = data.cbegin(); // begin of message
    std::string::const_iterator iter = msg_begin; // Index in data buffer
    const std::string tina_separators = "\n\x02";

    if (PACKET_DEBUG) {
        ROS_ERROR_STREAM("dataInput: " << data << " d:" << tina_package_depth);
    }

    // search for tina packages with cmenu outputs in between
    while (iter < data.end()) {
        iter = std::find_first_of(iter, data.cend(), tina_separators.cbegin(), tina_separators.cend());

        // append data to buffer until separator or end
        if (iter < data.cend()) {
            if (!packageBuffer_.size()) {
                packageBuffer_.push_back("");
            }
            packageBuffer_.back().append({msg_begin, iter});

            if (*iter == '\x02') {
                ++tina_package_depth;
                packageBuffer_.push_back("");
            } else { // if (*iter == '\n')
                auto singlebuf = trimmedBuffer(packageBuffer_.back().cbegin(), packageBuffer_.back().cend());
                handlePacketShell(singlebuf);
                --tina_package_depth;
                packageBuffer_.pop_back();
            }
            ++iter;
            msg_begin = iter;
        }
    }
}

void CRTBotHWInterface::handlePacketTina(std::string line)
{
    if (!line.size()) {
        return;
    }

    if (PACKET_DEBUG) {
        ROS_INFO_STREAM("pkg tina: " << line);
    }
}

void CRTBotHWInterface::handlePacketShell(std::string line)
{
    if (!line.size()) {
        return;
    }

    if (PACKET_DEBUG) {
        ROS_INFO_STREAM("pkg shell: " << line);
    }

    std::vector<std::string> parts;
    tokenize<std::vector<std::string>>(line, parts);

    if (parts[0] != "roscomm_answer:") {
        // not a roscomm response
        ds_serial.write("\x02" + line + "\r\n");
        return;
    }
    // we received something, don't wait for more responses even if data invalid
    got_response = true;

    static constexpr unsigned RX_VALUES = 8;
    if (parts.size() < RX_VALUES) {
        ROS_ERROR_STREAM("serial read parser fail: parts size");
        return;
    }
    // magic makes sure data is complete (nothing cut in the last number).
    // handle different \r\n at the end
    if (parts[RX_VALUES-1].size() < strlen("MAGIC") || parts[RX_VALUES-1].substr(0, strlen("MAGIC")) != "MAGIC") {
        ROS_ERROR_STREAM("serial read parser fail: magic");
        return;
    }

    // parse all values
    float x, y, phi, vtrans, vrot, batv;
    try {
        unsigned i = 1;
        x = std::stod(parts[i++]) / 1000.0;
        y = std::stod(parts[i++]) / 1000.0;
        phi = std::stod(parts[i++]);
        vtrans = std::stod(parts[i++]);
        vrot = std::stod(parts[i++]);
        batv = std::stod(parts[i++]);
    } catch (std::invalid_argument) {
        ROS_ERROR_STREAM("problem with (" << line << ")");
        return;
    }

    (void)vtrans;
    (void)vrot;

    // position (z always 0)
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0.0) );

    // orientation (only around z axis)
    tf::Quaternion q;
    q.setRPY(0, 0, phi);
    transform.setRotation(q);

    tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    // measurements
    sensor_msgs::BatteryState b;
    b.voltage = batv;
    battery_pub.publish(b);
//    ROS_ERROR("tf ok");
}

void CRTBotHWInterface::PrepareCb(const std_msgs::EmptyConstPtr&)
{
    request_prepare = true;
}

void CRTBotHWInterface::CleanWheelsCb(const std_msgs::EmptyConstPtr&)
{
    request_cleanwheels = true;
}

}  // namespace
