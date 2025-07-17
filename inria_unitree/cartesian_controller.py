#!/usr/bin/env python3

import os
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import pinocchio as pin
from pinocchio import SE3

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton
from PyQt5 import QtCore

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
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


class RightArmIKController(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'right_arm_ik_controller')
        QWidget.__init__(self)
        self.setWindowTitle("Right Arm IK & E-STOP")

        self.using_robot = self.declare_parameter('use_robot', False).value
        self.iface = self.declare_parameter('interface', 'eth0').value

        self.current = [0.0] * len(ALL_JOINT_INDICES)
        self.targets = [0.0] * len(ALL_JOINT_INDICES)
        self.smoothed = [0.0] * len(ALL_JOINT_INDICES)
        self.emergency_stop = False
        self.alpha = 0.2
        self.max_dq = 0.2
        self.received_unitree = False
        self.received_joint = False
        self.update_mode_machine_ = False

        qos = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)

        if self.using_robot:
            self._init_unitree()

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, qos)
        self.create_subscription(PoseStamped, '/right_hand_goal', self._ik_target_cb, qos)

        pkg_share = get_package_share_directory('g1_description')
        urdf = os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof.urdf')
        mesh = os.path.join(pkg_share, 'description_files', 'meshes')
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf, package_dirs=[mesh], root_joint=pin.JointModelFreeFlyer()
        )
        self.data = pin.Data(self.model)
        self.q = pin.neutral(self.model)
        self.q_prev = self.q.copy()

        self.name_to_q_index = {}
        for j in range(1, self.model.njoints):
            if self.model.joints[j].nq == 1:
                nm = self.model.names[j]
                if nm in _joint_index_to_ros_name.values():
                    self.name_to_q_index[nm] = self.model.joints[j].idx_q
        self.actuated_idx = sorted(self.name_to_q_index.values())

        self.eff_frame = 'right_hand_point_contact'
        self.eff_frame_id = self.model.getFrameId(self.eff_frame)

        layout = QVBoxLayout(self)
        self.estop_btn = QPushButton("EMERGENCY STOP")
        self.estop_btn.setStyleSheet("font-size: 24px; background-color: red; color: white;")
        self.estop_btn.clicked.connect(self._on_estop)
        layout.addWidget(self.estop_btn)

        self._init_timer = self.create_timer(5.0, self._on_init_timeout)
        self._qt_spin = QtCore.QTimer(self)
        self._qt_spin.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0))
        self._qt_spin.start(10)

    def _init_unitree(self):
        ChannelFactoryInitialize(0, self.iface)
        self.robot = LocoClient(); self.robot.SetTimeout(10.0); self.robot.Init()
        self.msc = MotionSwitcherClient(); self.msc.SetTimeout(5.0); self.msc.Init()
        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        self.lowstate_sub = ChannelSubscriber('rt/lowstate', LowStateType)
        self.lowstate_sub.Init(self._lowstate_cb, 10)
        self.cmd_pub = ChannelPublisher('rt/lowcmd', LowCmdType); self.cmd_pub.Init()
        self.crc = CRC()

    def _on_init_timeout(self):
        if not self.received_joint:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(_joint_index_to_ros_name.values())
            js.position = [0.0] * len(_joint_index_to_ros_name)
            self.joint_pub.publish(js)
            self.get_logger().info('Published zero joint states')
        self.destroy_timer(self._init_timer)

    def _lowstate_cb(self, msg: LowStateType):
        for i in ALL_JOINT_INDICES:
            self.current[i] = msg.motor_state[i].q
        if not self.received_unitree:
            self.targets = self.current.copy()
            self.smoothed = self.current.copy()
        if not self.update_mode_machine_:
            self.mode_machine_ = msg.mode_machine
            self.update_mode_machine_ = True
        self.received_unitree = True

    def _joint_state_cb(self, msg: JointState):
        if self.emergency_stop:
            return
        self.received_joint = True
        for name, pos in zip(msg.name, msg.position):
            if name in self.name_to_q_index:
                self.q[self.name_to_q_index[name]] = pos

    def _ik_target_cb(self, msg: PoseStamped):
        if self.emergency_stop:
            return
        p, o = msg.pose.position, msg.pose.orientation
        quat = pin.Quaternion(o.w, o.x, o.y, o.z)
        target = SE3(quat.matrix(), np.array([p.x, p.y, p.z]))
        q_sol = self._solve_ik(target)
        if q_sol is None:
            return
        dq = np.clip(q_sol - self.q_prev, -self.max_dq, self.max_dq)
        q_step = self.q_prev + dq
        self.q = (1 - self.alpha) * self.q_prev + self.alpha * q_step
        self.q_prev = self.q.copy()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(_joint_index_to_ros_name.values())
        js.position = [float(self.q[self.name_to_q_index.get(n, 0)]) for n in _joint_index_to_ros_name.values()]
        self.joint_pub.publish(js)

    def _solve_ik(self, target: SE3, max_iter=50, tol=1e-4, damping=1e-6):
        q = self.q.copy()
        free_nv = sum(j.nv for j in self.model.joints if j.nq > 1)
        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            M_cur = self.data.oMf[self.eff_frame_id]
            err_tf = M_cur.inverse() * target
            err = pin.log(err_tf).vector
            if np.linalg.norm(err) < tol:
                return q
            J6 = pin.computeFrameJacobian(self.model, self.data, q,
                                            self.eff_frame_id,
                                            pin.LOCAL_WORLD_ALIGNED)
            J_red = J6[:, free_nv:]
            JJt = J_red @ J_red.T
            dq = J_red.T @ np.linalg.solve(JJt + damping * np.eye(6), err)
            q[self.actuated_idx] += dq
        self.get_logger().warn("IK did not converge")
        return None

    def _on_estop(self):
        self.get_logger().error("Emergency stop triggered")
        if not self.using_robot:
            self.emergency_stop = True
            return
        cmd = LowCmdMessage()
        cmd.mode_pr = Mode.PR
        cmd.mode_machine = self.mode_machine_
        cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1
        for i in ALL_JOINT_INDICES:
            m = cmd.motor_cmd[i]
            m.mode, m.q, m.dq, m.tau, m.kp, m.kd = 0, 0.0, 0.0, 0.0, 60.0, 1.5
        cmd.crc = self.crc.Crc(cmd)
        self.cmd_pub.Write(cmd)
        self.emergency_stop = True


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    widget = RightArmIKController()
    widget.show()
    app.exec_()
    widget.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
