#!/usr/bin/env python
# Crash's Eurobot 2017 System Control
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

import copy
import rospy
import smach
import smach_ros
import tf
import multi_static_if
import dynamic_reconfigure.client
from numpy.core.umath import deg2rad
import math
from tf import transformations as t

from std_msgs.msg import Empty, Bool, String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import std_srvs.srv

def norm_angle(x):
    return x % (2*math.pi)

class Pitch:
    def __init__(self):
        self.teamcolor = None
        self.gamestart = None

    def time_elapsed(self):
        return rospy.Time.now() - self.gamestart

    def game_over(self):
        t = self.time_elapsed()
        return t.secs >= 90

PITCH = Pitch()

def LedSet(val, topic="/arduino/lighthouse"):
    pub = rospy.Publisher(topic, String, queue_size=10, latch=True)
    pub.publish(String(val))

def WaitForPoseReset_cb(ud, msg):
    if msg.data == True:
        rospy.loginfo("pose reset pressed")
        return False

    return True

def lookupTransform(parent, child):
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            return listener.lookupTransform(parent, child, rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    return None

def transRotToMatrix(transRot):
    return t.concatenate_matrices(t.translation_matrix(transRot[0]), t.quaternion_matrix(transRot[1]))

def matrixToTransRot(mat):
    return (t.translation_from_matrix(mat), t.quaternion_from_matrix(mat))

class SetPoseToOrigin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok','preempted','failed'])
        self.prepare_pub = rospy.Publisher("/crtbot/prepare", Empty)

    def execute(self, userdata):
        LedSet("rwrwr")

        # reset bot pose in LMC
        msg = Empty()
        self.prepare_pub.publish(msg)

        tries = 0
        max_tries = 3

        # wait for success
        while not rospy.is_shutdown() and tries < max_tries:
            mypos = lookupTransform("/world", "/base_link")
            if not mypos:
                break
            trans, rot = mypos

            # in m
            tolerance = 5e-3
            bot_length_back = 74e-3

            if abs(trans[0]) < tolerance and abs(trans[1] - bot_length_back) < tolerance:
                break
            else:
                rospy.loginfo("waiting for correct pose")
                rospy.sleep(0.2)

            tries += 1

        if rospy.is_shutdown():
            return 'preempted'

        if tries >= max_tries:
            rospy.logerror("pose reset failed")
            return 'failed'

        rospy.loginfo("pose reset done")
        return 'ok'

class WaitForMoveToStartCorner(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok','preempted'])

    def execute(self, userdata):
        LedSet("wwwww")
        teamcolor = "x"

        while not rospy.is_shutdown():
            mypos = lookupTransform("/world", "/base_link")
            if not mypos:
                break
            trans, rot = mypos

            teamcolor_distance = 10e-2 # 10 cm

            if abs(trans[0]) > teamcolor_distance:
                # teamcolor set

                if trans[0] > 0.0:
                    teamcolor = "b"
                else:
                    teamcolor = "y"

                break

            rospy.sleep(0.1)

        if rospy.is_shutdown():
            return 'preempted'

        rospy.loginfo("we are inside start corner of team '%s'" % teamcolor)
        LedSet(teamcolor*5)
        PITCH.teamcolor = teamcolor
        return 'ok'

def WaitForStartJumperInsert_cb(ud, msg):
    if msg.data == True:
        rospy.loginfo("start jumper inserted")
        return False

    return True

def WaitForStartJumperPull_cb(ud, msg):
    if msg.data == False:
        rospy.loginfo("start jumper pulled")
        PITCH.gamestart = rospy.Time.now()
        return False

    return True

class MoveBaseState(smach_ros.SimpleActionState):
    """Calls a move_base action server with the goal (x, y, yaw) from userdata
    source: https://github.com/felix-kolbe/uashh-rvl-ros-pkg/blob/master/uashh_smach/src/uashh_smach/platform/move_base.py"""

    def __init__(self, **kwargs):
        smach_ros.SimpleActionState.__init__(self, 'move_base', MoveBaseAction, goal_cb=self.__goal_cb, result_cb=self.__result_cb)
        self.frame = kwargs.get("frame", "/map")
        self.target_pose = list(kwargs.get("target"))
        self.mirror_sides = kwargs.get("mirror_sides", False)
        self.mirror_angle = kwargs.get("mirror_angle", False)
        self.disable_obstacles = kwargs.get("disable_obstacles", False)

    def __result_cb(self, userdata, status, result):
        if PITCH.game_over():
            return 'preempted'

    def __goal_cb(self, userdata, old_goal):
        clientL = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
        clientG = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
        clientL.update_configuration({"enabled": not self.disable_obstacles})
        clientG.update_configuration({"enabled": not self.disable_obstacles})
        if self.disable_obstacles:
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)

        target_pose = copy.copy(self.target_pose)
        if self.mirror_sides and PITCH.teamcolor == "y":
            target_pose[0] = -target_pose[0]

            if self.mirror_angle:
                target_pose[2] = norm_angle(deg2rad(180) - target_pose[2])

        rospy.loginfo("driving to target %s (%s,%s,%s) obst:%s", target_pose, self.mirror_sides, self.mirror_angle, PITCH.teamcolor, self.disable_obstacles)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = tf.transformations.quaternion_from_euler(0, 0, target_pose[2])
        goal.target_pose.pose.orientation = Quaternion(*quat)
        goal.target_pose.pose.position = Point(target_pose[0], target_pose[1], 0)
        return goal

class GameStart(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok'])
        self.abort_navigation_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        self.rocket_pub = rospy.Publisher("/arduino/rocket", Bool)

    def execute(self, userdata):
        rospy.loginfo("game started, we are team '%s'" % PITCH.teamcolor)
        rospy.Timer(rospy.Duration(90), self.end_of_game_cb, oneshot=True)
        clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.srv.Empty)
        return 'ok'

    def end_of_game_cb(self, event):
        rospy.loginfo("Cancelling move_base actions")
        self.abort_navigation_pub.publish(GoalID())

        rospy.loginfo("eog: launching rocket")
        msg = Bool()
        msg.data = True
        self.rocket_pub.publish(msg)
        rospy.loginfo("eog: done")

class LaunchRocket(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok'])
        self.rocket_pub = rospy.Publisher("/arduino/rocket", Bool)

    def execute(self, userdata):
        t = PITCH.time_elapsed()
        if t.secs <= 91:
            slp = 91 - t.secs
            rospy.loginfo("rocket: %d s until after game end" % slp)
            rospy.sleep(slp)

        rospy.loginfo("rocket: launching")
        msg = Bool()
        msg.data = True
        self.rocket_pub.publish(msg)
        rospy.loginfo("rocket: done")
        return 'ok'

class GameReset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok'])

    def execute(self, userdata):
        PITCH = Pitch()
        LedSet("gxxxg")
        rospy.loginfo("game reset")
        return 'ok'

def main():
    rospy.init_node("crash_sc")

    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('WaitForPoseReset', smach_ros.MonitorState("/arduino/up", Bool, WaitForPoseReset_cb), transitions={'invalid':'SetPoseToOrigin', 'valid':'WaitForPoseReset', 'preempted':'WaitForPoseReset'})
        smach.StateMachine.add('SetPoseToOrigin', SetPoseToOrigin(), transitions={'ok':'WaitForMoveToStartCorner', 'preempted':'DONE', 'failed':'SetPoseToOrigin'})
        smach.StateMachine.add('WaitForMoveToStartCorner', WaitForMoveToStartCorner(), transitions={'ok':'WaitForStartJumperInsert', 'preempted':'DONE'})
        smach.StateMachine.add('WaitForStartJumperInsert', smach_ros.MonitorState("/arduino/start", Bool, WaitForStartJumperInsert_cb), transitions={'invalid':'WaitForStartJumperPull', 'valid':'WaitForStartJumperInsert', 'preempted':'WaitForStartJumperInsert'})
        smach.StateMachine.add('WaitForStartJumperPull', smach_ros.MonitorState("/arduino/start", Bool, WaitForStartJumperPull_cb), transitions={'invalid':'GameStart', 'valid':'WaitForStartJumperPull', 'preempted':'WaitForStartJumperPull'})
        smach.StateMachine.add('GameStart', GameStart(), transitions={'ok':'MoveLunarEntry'})
        smach.StateMachine.add('MoveLunarEntry', MoveBaseState(target=(0.5, 0.92, deg2rad(-90)), mirror_sides=True, mirror_angle=True), transitions={'preempted':'LaunchRocket','succeeded':'MoveLunarPush','aborted':'MoveLunarEntry'})
        smach.StateMachine.add('MoveLunarPush', MoveBaseState(target=(0.5, 0.36, deg2rad(-135)), mirror_sides=True, mirror_angle=True), transitions={'preempted':'LaunchRocket','succeeded':'LaunchRocket','aborted':'MoveLunarPush'})
        smach.StateMachine.add('LaunchRocket', LaunchRocket(), transitions={'ok':'GameReset'})
        smach.StateMachine.add('GameReset', GameReset(), transitions={'ok':'WaitForPoseReset'})

#    sis = smach_ros.IntrospectionServer('smach_server', sm, 'sm_root')
#    sis.start()
    sm.execute()
    rospy.spin()
#    sis.stop()

if __name__=="__main__":
    main()
