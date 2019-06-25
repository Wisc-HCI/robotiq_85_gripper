#!/usr/bin/env python

'''
URScript Gripper Action Server
Author: Curt Henrichs
Date: 6-25-19

Developed to provide control for UR3e's embedded Robotiq gripper.

This action server provides a standard GripperCommand interface to control the
a Robotiq gripper through URScript running on a UR robot through the
ur_modern_driver's /urscript topic.

Parameters:
    - ~speed
        "Gripper movement speed"
        default = 255
    - ~urscript_topic
        "Name of topic to publish generated UR script"
        default = '/ur_driver/URScript'
    - ~run_delay
        "Used to provide a wait behavior since there is currently no feedback"
        default = 5

Actions:
    - /gripper/gripper_command
        Type = control_msgs/GripperCommand

Topics Published
    - <urscript_topic>
        Type = std_msgs/String
'''
#TODO this server is not complete as it does not have feedback from the UR
# program for when it is completed. Perhaps an undergrad would like to polish up
# the driver?

import os
import time
import rospy
import actionlib

from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandFeedback, GripperCommandResult


DEFAULT_SPEED = 255
DEFAULT_URSCRIPT_TOPIC = '/ur_driver/URScript'
DEFAULT_RUN_DELAY = 5

DELAY_TIMESTEP = 0.01

GRIPPER_FILE = os.path.join(os.path.realpath(__file__),'urscript/gripper_all_in_one.script')


class URScriptGripperActionServer:

    def __init__(self,speed,run_delay,urscript_topic):
        self._speed = speed
        self._run_delay = run_delay

        self._joint_state_pub = rospy.Publisher('/gripper/joint_states',JointState,queue_size=10)
        self._urscript_pub = rospy.Publisher(urscript_topic,String,queue_size=5)
        self._action_server = actionlib.SimpleActionServer('gripper_command', GripperCommandAction, self._execute, False)
        self._action_server.start()

    def _execute(self, goal):

        # verify parameters are valid
        #TODO convert from goal in nominal scale to byte scale
        if not (0 <= goal.position <= 255):
            self._action_server.set_aborted(self._fail_res_msg())
            return
        elif not (0 <= goal.max_effort <= 255):
            self._action_server.set_aborted(self._fail_res_msg())
            return
        elif not (0 <= self._speed <= 255):
            self._action_server.set_aborted(self._fail_res_msg())
            return

        # send script to UR driver
        msg = String(self._grip_script(position=goal.position,
                                       speed=self._speed,
                                       force=goal.max_effort))
        self._urscript_pub.publish(msg)

        # TODO remove this preset when actual feedback developed
        fb_position = goal.position
        fb_effort = goal.max_effort
        fb_stalled = False

        # wait for the alloted time
        preempted = False
        start_time = time.time()
        while (time.time() - start_time) > self._run_delay:
            if self._action_server.is_preempt_requested():
                preempted = True
                break

            rospy.sleep(DELAY_TIMESTEP)

            # TODO get actual feedback data from a better gripper script
            # fb_position = ?
            # fb_effort = ?
            # fb_stalled = ?
            self._action_server.publish_feedback(self._feedback_msg(goal.position,fb_position,fb_effort,fb_stalled))

        # final result
        result = self._std_res_msg(goal.position,fb_position,fb_effort,fb_stalled)
        if preempted:
            self._action_server.set_preempted(result)
        else:
            self._action_server.set_succeeded(result)

    def _fail_res_msg(self):
        result = GripperCommandResult()
        result.reached_goal = False
        return result

    def _std_res_msg(self,position_target,position_actual,effort_actual,stalled):
        result = GripperCommandResult()
        result.position = position_actual
        result.effort = effort_actual
        result.stalled = stalled
        result.reached_goal = abs(position_target - position_actual) < self._grip_threshold
        return result

    def _feedback_msg(self,position_target,position_actual,effort_actual,stalled):
        feedback = GripperCommandFeedback()
        feedback.position = position_actual
        feedback.effort = effort_actual
        feedback.stalled = stalled
        feedback.reached_goal = abs(position_target - position_actual) < self._grip_threshold
        return feedback

    def _gen_script(self,position,speed,force):
        # Note requires the file to be parameterized

        fin = open(GRIPPER_FILE,'r')
        script = fin.read()
        fin.close()

        script = script.replace('<position>',str(position))
        script = script.replace('<speed>',str(speed))
        script = script.replace('<force>',str(force))
        return script

    def loop(self):
        #TODO publish actual gripper state
        msg = JointState()
        msg.name= ['robotiq_85_left_knuckle_joint']
        msg.position = [0]
        msg.velocity = [0]
        msg.effort = [0]
        self._joint_state_pub.Publish(msg)


if __name__ == "__main__":
    rospy.init_node('urscript_gripper_action_server')
    speed = rospy.get_param('~speed',DEFAULT_SPEED)
    urscript_topic = rospy.get_param('~urscript_topic',DEFAULT_URSCRIPT_TOPIC)
    run_delay = rospy.get_param('~run_delay',DEFAULT_RUN_DELAY)
    server = URScriptGripperActionServer(speed,run_delay,urscript_topic)
    rospy.spin()
