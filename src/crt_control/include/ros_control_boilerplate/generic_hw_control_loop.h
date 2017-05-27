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
   Desc:   Example control loop for reading, updating, and writing commands to a hardware interface
   using MONOTOIC system time
*/

#include <time.h>
#include <ros_control_boilerplate/generic_hw_interface.h>

namespace ros_control_boilerplate
{
// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;

/**
 * \brief The control loop - repeatidly calls read() and write() to the hardware interface at a
 * specified frequency
 *        We use MONOTONIC time to ensure robustness in the event of system time updates/change.
 *        See
 * http://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
 */
class GenericHWControlLoop
{
public:
  /**
   * \brief Constructor
   * \param NodeHandle
   * \param hardware_interface - the robot-specific hardware interface to be use with your robot
   */
  GenericHWControlLoop(
      ros::NodeHandle& nh,
      boost::shared_ptr<ros_control_boilerplate::GenericHWInterface> hardware_interface);

  /** \brief Timer event
   *         Note: we do not use the TimerEvent time difference because it does NOT guarantee that
   * the time source is
   *         strictly linearly increasing
   */
  void update(const ros::TimerEvent& e);

protected:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Name of this class
  std::string name_ = "generic_hw_control_loop";

  // Settings
  ros::Duration desired_update_freq_;
  double cycle_time_error_threshold_;

  // Timing
  ros::Timer non_realtime_loop_;
  ros::Duration elapsed_time_;
  double loop_hz_;
  struct timespec last_time_;
  struct timespec current_time_;

  /** \brief ROS Controller Manager and Runner
   *
   * This class advertises a ROS interface for loading, unloading, starting, and
   * stopping ros_control-based controllers. It also serializes execution of all
   * running controllers in \ref update.
   */
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  /** \brief Abstract Hardware Interface for your robot */
  boost::shared_ptr<ros_control_boilerplate::GenericHWInterface> hardware_interface_;

};  // end class

}  // namespace
