#!/usr/bin/env python3

import os
import sys
import math
import time

os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets, QtCore

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
    ChannelPublisher,
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdMessage
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdType, LowState_ as LowStateType
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class Mode:
    PR = 0
    AB = 1


class G1JointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleRoll = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleRoll = 11
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
    kNotUsedJoint = 29


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
RIGHT_JOINT_INDICES = [
    G1JointIndex.RightShoulderPitch,
    G1JointIndex.RightShoulderRoll,
    G1JointIndex.RightShoulderYaw,
    G1JointIndex.RightElbow,
    G1JointIndex.RightWristRoll,
    G1JointIndex.RightWristPitch,
    G1JointIndex.RightWristYaw,
]


class JointController(Node, QtWidgets.QWidget):
    def __init__(self):
        Node.__init__(self, "combined_controller")
        QtWidgets.QWidget.__init__(self)
        self.setWindowTitle("Arm Control & Command")

        self.current = [0.0] * len(ALL_JOINT_INDICES)
        self.targets = [0.0] * len(ALL_JOINT_INDICES)
        self.smoothed = [0.0] * len(ALL_JOINT_INDICES)
        self.received = False
        self.mode_machine_ = 0
        self.update_mode_machine_ = False
        self.is_estop = False

        self.alpha = 0.2
        self.using_robot = self.declare_parameter(
            "use_robot", False
        ).get_parameter_value().bool_value
        iface = self.declare_parameter("interface", "eth0").get_parameter_value().string_value

        if self.using_robot:
            try:
                ChannelFactoryInitialize(0, iface)
            except Exception as e:
                self.get_logger().warn(
                    f"Interface '{iface}' unavailable, falling back: {e}"
                )
                ChannelFactoryInitialize(0)

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

            self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowStateType)
            self.lowstate_sub.Init(self._lowstate_cb, 10)

            self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmdType)
            self.cmd_pub.Init()
            self.crc = CRC()
        else:
            self.received = True
            self.update_mode_machine_ = True

        qos = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", qos)

        layout = QtWidgets.QVBoxLayout(self)
        self.sliders = {}
        for idx in RIGHT_JOINT_INDICES:
            layout.addWidget(QtWidgets.QLabel(_joint_index_to_ros_name[idx]))
            sld = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            sld.setRange(-180, 180)
            sld.setValue(0)
            sld.valueChanged.connect(lambda deg, i=idx: self._on_slider(i, deg))
            layout.addWidget(sld)
            self.sliders[idx] = sld

        home_btn = QtWidgets.QPushButton("Home Position")
        home_btn.clicked.connect(self._go_home)
        layout.addWidget(home_btn)

        estop_btn = QtWidgets.QPushButton("E-STOP")
        estop_btn.setMinimumHeight(100)
        estop_btn.setStyleSheet("background-color: red; color: white;")
        estop_btn.clicked.connect(self._estop)
        layout.addWidget(estop_btn)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(50)

    def _lowstate_cb(self, msg: LowStateType):
        """Callback DDS LowState → actualiza posición real."""
        for i in ALL_JOINT_INDICES:
            self.current[i] = msg.motor_state[i].q
        if not self.received:
            self.targets = self.current.copy()
            self.smoothed = self.current.copy()
        if not self.update_mode_machine_:
            self.mode_machine_ = msg.mode_machine
            self.update_mode_machine_ = True
        self.received = True

    def _on_slider(self, idx: int, deg: int):
        self.targets[idx] = math.radians(deg)

    def _go_home(self):
        for idx, sld in self.sliders.items():
            sld.setValue(0)
            self.targets[idx] = 0.0

    def _estop(self):
        if not self.using_robot:
            self.is_estop = True
            return
        cmd = LowCmdMessage()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine_
        cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
        for i in ALL_JOINT_INDICES:
            m = cmd.motor_cmd[i]
            m.mode = 0
            m.q = 0.0
            m.dq = 0.0
            m.tau = 0.0
            m.kp = 60.0
            m.kd = 1.5
        cmd.crc = self.crc.Crc(cmd)
        self.cmd_pub.Write(cmd)
        self.is_estop = True

    def _tick(self):
        if not (self.received and self.update_mode_machine_):
            return

        for i in ALL_JOINT_INDICES:
            self.smoothed[i] += self.alpha * (self.targets[i] - self.smoothed[i])

        self._publish_ros()

        if self.using_robot:
            self._send_cmd(smoothed=True)

    def _publish_ros(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [_joint_index_to_ros_name[i] for i in ALL_JOINT_INDICES]
        js.position = [
            self.smoothed[i] if i in RIGHT_JOINT_INDICES else self.current[i]
            for i in ALL_JOINT_INDICES
        ]
        self.joint_pub.publish(js)

    def _send_cmd(self, *, smoothed: bool = False):
        if self.is_estop or not self.using_robot:
            return

        cmd = LowCmdMessage()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine_
        cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1

        for i in ALL_JOINT_INDICES:
            m = cmd.motor_cmd[i]
            if i in RIGHT_JOINT_INDICES:
                desired = self.smoothed[i] if smoothed else self.targets[i]
                m.mode = 1
            else:
                desired = self.current[i]
                m.mode = 0
            m.q = desired
            m.dq = 0.0
            m.tau = 0.0
            m.kp = 60.0
            m.kd = 1.5

        cmd.crc = self.crc.Crc(cmd)
        self.cmd_pub.Write(cmd)

def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    widget = JointController()
    widget.show()

    spinner = QtCore.QTimer()
    spinner.timeout.connect(lambda: rclpy.spin_once(widget, timeout_sec=0))
    spinner.start(10)

    app.exec_()
    widget.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
