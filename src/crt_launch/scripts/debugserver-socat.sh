#!/bin/sh
# This script starts the socat serial port relay to pipe data between the
# ros_control node and the TURAG Debug Server.
# Normally the Debug Server would connect to the LMC's serial port directly,
# but ros_control already needs it.
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

socat -d -d pty,raw,echo=0,link=/tmp/ttycrtros1,b1000000 pty,raw,echo=0,link=/tmp/ttycrtros2,b1000000
