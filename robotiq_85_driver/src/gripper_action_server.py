#!/usr/bin/env python

'''
Gripper Action Server
Author: Curt Henrichs
Date: 5-29-19

Developed to provide a standard control message interface for the robotiq 85
gripper.

This action server provides a standard interface for robot controllers to
interact with the robotiq gripper by making use of a GripperCommandAction.
GripperCommandAction was chosen as it provides interoperability with MoveIt while
still being general for other pipelines. The node generates gripper command
messages to communicate with robotiq 85 driver provided in the ROS package.

Note: A related project developed in C++ that was found after this was written,
https://github.com/ros-industrial/robotiq
Specifically, 2f_gripper_action_server

Parameters:
    - ~grip_threshold
        "Reached target position if error is less than this value"
        Default = 0.01
    - ~loop_rate
        "Action server execute loop feedback rate per second"
        Default = 20
    - ~speed
        "Gripper movement speed"
        Default = 0.1

Actions:
    - /gripper/gripper_command
        Type = control_msgs/GripperCommand

Topics Published:
    - /gripper/cmd
        Type = robotiq_85_msgs/GripperCmd

Topics Subscribed:
    - /gripper/stat
        Type = robotiq_85_msgs/GripperStat

'''
# ignore effort for now this should become the force attribute
#  if effort == 0, use a default effort

import roslib
roslib.load_manifest('m1n6s_robotiq85_moveit_config')
import rospy
import actionlib

from robotiq_85_msgs.msg import GripperCmd, GripperStat
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult


LOWER_LIMIT = 0.0
UPPER_LIMIT = 0.085

GRIP_THRESHOLD = 0.01
LOOP_RATE = 20
SPEED = 0.1


class GripperActionServer:

    def __init__(self,loop_rate,grip_threshold,speed):
        self._loop_delay = 1 / loop_rate
        self._grip_threshold = grip_threshold
        self._speed = speed

        self._statSub = rospy.Subscriber('/gripper/stat', GripperStat, self._update_gripper_stat, queue_size=10)
        self._cmdPub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)

        self._stat = GripperStat()

        self._action_server = actionlib.SimpleActionServer('gripper_command', GripperCommandAction, self._execute, False)
        self._action_server.start()

    def _update_gripper_stat(self, stat):
        self._stat = stat

    def _generate_result_msg(self, goal):

        print goal, 'vs', self._stat.position

        result = GripperCommandResult()
        result.position = self._stat.position
        result.effort = 0
        result.stalled = False
        result.reached_goal = abs(self._stat.position - goal) < self._grip_threshold
        return result

    def _generate_feedback_msg(self, goal):
        feedback = GripperCommandFeedback()
        feedback.position = self._stat.position
        feedback.effort = 0
        feedback.stalled = False
        feedback.reached_goal = abs(self._stat.position - goal) < self._grip_threshold
        return feedback

    def _execute(self, goal):
        goalPosition = UPPER_LIMIT - goal.command.position / 10.0

        # check gripper is ready to accept commands
        if not self._stat.is_ready:
            rospy.logerr('Gripper reporting not ready')
            self._action_server.set_aborted(self._generate_result_msg(goalPosition))
            return

        # verify position is valid
        if goalPosition < LOWER_LIMIT or goalPosition > UPPER_LIMIT:
            rospy.logerr('Gripper position out of range')
            self._action_server.set_aborted(self._generate_result_msg(goalPosition))
            return

        # send gripper move command
        cmd = GripperCmd()
        cmd.position = goalPosition
        cmd.speed = self._speed
        cmd.force = 100.0
        self._cmdPub.publish(cmd)
        rospy.sleep(0.5)

        # move gripper
        preempted = False
        while self._stat.is_moving:
            if self._action_server.is_preempt_requested():
                # send gripper halt command
                cmd = GripperCmd()
                cmd.stop = True
                self._cmdPub.publish(cmd)
                preempted = True
                break

            rospy.sleep(self._loop_delay)
            self._action_server.publish_feedback(self._generate_feedback_msg(goalPosition))

        # final result
        result = self._generate_result_msg(goalPosition)
        if preempted:
            self._action_server.set_preempted(result)
        elif result.reached_goal:
            self._action_server.set_succeeded(result)
        else:
            rospy.logwarn('Gripper did not reach goal position')
            self._action_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('gripper_action_server')

    loop_rate = rospy.get_param('~loop_rate',LOOP_RATE)
    grip_threshold = rospy.get_param('~grip_threshold',GRIP_THRESHOLD)
    speed = rospy.get_param('~speed',SPEED)

    server = GripperActionServer(loop_rate,grip_threshold,speed)
    rospy.spin()
