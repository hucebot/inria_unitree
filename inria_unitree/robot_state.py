#!/usr/bin/env python3

import os
import sys
import math
import time

os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from astroviz_interfaces.msg import MotorState, MotorStateList

from PyQt5 import QtWidgets, QtCore

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber
)

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateType
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


_joint_index_to_ros_name = {
    0: "left_hip_pitch_joint",
    1: "left_hip_roll_joint",
    2: "left_hip_yaw_joint",
    3: "left_knee_joint",
    4: "left_ankle_pitch_joint",
    5: "left_ankle_roll_joint",
    6: "right_hip_pitch_joint",
    7: "right_hip_roll_joint",
    8: "right_hip_yaw_joint",
    9: "right_knee_joint",
    10: "right_ankle_pitch_joint",
    11: "right_ankle_roll_joint",
    12: "waist_yaw_joint",
    13: "waist_roll_joint",
    14: "waist_pitch_joint",
    15: "left_shoulder_pitch_joint",
    16: "left_shoulder_roll_joint",
    17: "left_shoulder_yaw_joint",
    18: "left_elbow_joint",
    19: "left_wrist_roll_joint",
    20: "left_wrist_pitch_joint",
    21: "left_wrist_yaw_joint",
    22: "right_shoulder_pitch_joint",
    23: "right_shoulder_roll_joint",
    24: "right_shoulder_yaw_joint",
    25: "right_elbow_joint",
    26: "right_wrist_roll_joint",
    27: "right_wrist_pitch_joint",
    28: "right_wrist_yaw_joint",
}

ALL_JOINT_INDICES = list(range(29))


class RobotState(Node, QtWidgets.QWidget):
    def __init__(self):
        Node.__init__(self, "robot_state")
        QtWidgets.QWidget.__init__(self)
        self.setWindowTitle("Robot State")

        iface = self.declare_parameter("interface", "eth0").get_parameter_value().string_value

        ChannelFactoryInitialize(0, iface)

        self.current = [0.0] * len(ALL_JOINT_INDICES)

        self.robot = LocoClient()
        self.robot.SetTimeout(10.0)
        self.robot.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        status, result = self.msc.CheckMode()
        while result["name"]:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

        self.motor_state_pub = self.create_publisher(MotorStateList, "/inria_unitree/motor_state", QoSProfile(depth=10))

        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowStateType)
        self.lowstate_sub.Init(self._lowstate_cb, 10)

    def _lowstate_cb(self, msg: LowStateType):
        motor_list_msg = MotorStateList()
        for i in ALL_JOINT_INDICES:
            motor_state = MotorState()
            motor_state.name = _joint_index_to_ros_name[i]
            motor_state.temperature = float(msg.motor_state[i].temperature[0])
            motor_state.voltage = float(msg.motor_state[i].vol)
            motor_state.position = float(msg.motor_state[i].q)
            motor_state.velocity = float(msg.motor_state[i].dq)
            motor_list_msg.motor_list.append(motor_state)
        self.motor_state_pub.publish(motor_list_msg)


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    widget = RobotState()
    widget.show()

    spinner = QtCore.QTimer()
    spinner.timeout.connect(lambda: rclpy.spin_once(widget, timeout_sec=0))
    spinner.start(10)

    app.exec_()
    widget.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
