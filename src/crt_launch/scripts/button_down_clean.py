#!/usr/bin/python
# Send CLEAN_WHEELS request to LMC (via the ros_control node) when the UP button
# of the UI Arduino is pressed.
# Copyright (C) 2017  Crash Racing Team <crt@turag.de>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from std_msgs.msg import Empty, Bool

def down_cb(data):
    pub = rospy.Publisher("/crtbot/clean_wheels", Empty, queue_size=1)
    if data.data:
        pub.publish()
        rospy.loginfo("cleaning wheels")

def listener():
    rospy.init_node("button_up_clean")
    rospy.Subscriber("/arduino/down", Bool, down_cb)
    rospy.spin()

if __name__ == "__main__":
    listener()
