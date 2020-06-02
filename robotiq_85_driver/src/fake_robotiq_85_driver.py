#!/usr/bin/env python

from robotiq_85_msgs.msg import GripperCmd, GripperStat
from sensor_msgs.msg import JointState
import numpy as np
import rospy


DELAY_RESPONSE_TIME = 0.25


class FakeDriver:

    def __init__(self):
        self._last_time = rospy.Time.now()

        rospy.Subscriber("/gripper/cmd", GripperCmd, self._update_gripper_cmd, queue_size=10)
        self._gripper_pub = rospy.Publisher('/gripper/stat', GripperStat, queue_size=10)
        self._gripper_joint_state_pub = rospy.Publisher('/gripper/joint_state', JointState, queue_size=10)

        self._seq = 0
        self._prev_js_position = 0.0
        self._prev_js_time = rospy.get_time()
        self._driver_state = 0
        self._driver_ready = True

    def _update_gripper_cmd(self,cmd):
        rospy.sleep(DELAY_RESPONSE_TIME)
        self._update_gripper_joint_state(cmd.position)
        self._update_gripper_stat(cmd.position)

    def _update_gripper_stat(self,position):
        stat = GripperStat()
        stat.header.stamp = rospy.get_rostime()
        stat.header.seq = self._seq
        stat.is_ready = self._driver_ready
        stat.is_reset = False
        stat.is_moving = False
        stat.obj_detected = False
        stat.fault_status = self._driver_state
        stat.position = position
        stat.requested_position = position
        stat.current = 0
        self._seq += 1
        return stat

    def _update_gripper_joint_state(self,position):
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = self._seq
        js.name = ['robotiq_85_left_knuckle_joint']
        js.position = [position]
        dt = rospy.get_time() - self._prev_js_time
        self._prev_js_time = rospy.get_time()
        js.velocity = [(position-self._prev_js_position)/dt]
        self._prev_js_position = position
        return js


if __name__ == "__main__":
    rospy.init_node('fake_driver')
    driver = FakeDriver()
    rospy.spin()
