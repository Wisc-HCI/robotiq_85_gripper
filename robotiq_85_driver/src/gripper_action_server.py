#!/usr/bin/env python
import roslib
roslib.load_manifest('m1n6s_robotiq85_moveit_config')
import rospy
import actionlib

from robotiq_85_msgs.msg import GripperCmd, GripperStat
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult


LOWER_LIMIT = 0.0
UPPER_LIMIT = 0.085


class GripperActionServer:

    def __init__(self):

        self._statSub = rospy.Subscriber('/gripper/stat', GripperStat, self._update_gripper_stat, queue_size=10)
        self._cmdPub = rospy.Publisher('/gripper/cmd', GripperCmd, queue_size=10)

        self._stat = GripperStat()

        self._action_server = actionlib.SimpleActionServer('gripper/gripper_command', GripperCommandAction, self._execute, False)
        self._action_server.start()

    def _update_gripper_stat(self, stat):
        self._stat = stat

    def _generate_result_msg(self, goal):
        result = GripperCommandResult()
        result.position = self._stat.position
        result.effort = 0
        result.stalled = False
        result.reached_goal = self._stat.position == goal
        return result

    def _generate_feedback_msg(self, goal):
        feedback = GripperCommandFeedback()
        feedback.position = self._stat.position
        feedback.effort = 0
        feedback.stalled = False
        feedback.reached_goal = self._stat.position == goal
        return feedback

    def _execute(self, goal):
        print '\n\n\n\n', goal, '\n\n\n\n'
        goalPosition = UPPER_LIMIT - goal.command.position / 10.0
        # ignore effort for now

        # check gripper is ready to accept commands
        if not self._stat.is_ready:
            self._action_server.set_aborted(self._generate_result_msg(goalPosition))
            return

        # verify position is valid
        if goalPosition < LOWER_LIMIT or goalPosition > UPPER_LIMIT:
            self._action_server.set_aborted(self._generate_result_msg(goalPosition))
            return

        # send gripper command
        cmd = GripperCmd()
        cmd.position = goalPosition
        cmd.speed = 0.02
        cmd.force = 100.0
        self._cmdPub.publish(cmd)
        rospy.sleep(0.05)

        # move gripper
        preempted = False
        while self._stat.is_moving:
            if self._action_server.is_preempt_requested():
                cmd = GripperCmd()
                cmd.stop = True
                self._cmdPub.publish(cmd)
                preempted = True
                break

            rospy.sleep(0.05)
            self._action_server.publish_feedback(self._generate_feedback_msg(goalPosition))

        # final result
        result = self._generate_result_msg(goalPosition)
        if preempted:
            self._action_server.set_preempted(result)
        elif result.reached_goal:
            self._action_server.set_succeeded(result)
        else:
            self._action_server.set_aborted(result)

if __name__ == "__main__":
    rospy.init_node('gripper_action_server')
    server = GripperActionServer()
    rospy.spin()
