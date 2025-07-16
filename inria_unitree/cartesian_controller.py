#!/usr/bin/env python3

import os
import time
import threading
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import pinocchio as pin
from pinocchio import SE3

import meshcat
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton

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

class RightArmIKController(Node):
    def __init__(self):
        super().__init__('right_arm_ik_controller')
        qos = QoSProfile(depth=10)

        self.using_robot = self.declare_parameter('use_robot', False).value
        self.iface       = self.declare_parameter('interface', 'eth0').value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)

        self._received_joint = False
        self._init_done      = False

        self.alpha  = 0.2
        self.max_dq = 0.05

        self.emergency_stop = False

        if self.using_robot:
            self._init_unitree()

        self.create_subscription(JointState,  '/joint_states', self._joint_state_cb, qos)
        self.create_subscription(PoseStamped, '/right_hand_goal', self._ik_target_cb,   qos)

        pkg_share = get_package_share_directory('g1_description')
        urdf_path = os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof.urdf')
        mesh_dir  = os.path.join(pkg_share, 'description_files', 'meshes')
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path,
            package_dirs=[mesh_dir],
            root_joint=pin.JointModelFreeFlyer()
        )
        self.data = pin.Data(self.model)

        self.q = pin.neutral(self.model)
        self.q_prev = self.q.copy()

        self.name_to_q_index = {}
        for j in range(1, self.model.njoints):
            if self.model.joints[j].nq == 1:
                name = self.model.names[j]
                if name in _joint_index_to_ros_name.values():
                    self.name_to_q_index[name] = self.model.joints[j].idx_q
        self.actuated_idx = sorted(self.name_to_q_index.values())

        self.all_joint_names = [
            _joint_index_to_ros_name[i] for i in range(len(_joint_index_to_ros_name))
        ]
        self.right_joint_names = {n for n in self.all_joint_names if n.startswith("right_")}

        self.eff_frame    = 'right_hand_point_contact'
        self.eff_frame_id = self.model.getFrameId(self.eff_frame)

        self._init_timer = self.create_timer(5.0, self._on_init_timeout)

    def _init_unitree(self):
        ChannelFactoryInitialize(0, self.iface)
        self.robot = LocoClient(); self.robot.SetTimeout(10.0); self.robot.Init()
        self.msc   = MotionSwitcherClient(); self.msc.SetTimeout(5.0); self.msc.Init()
        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        self.lowstate_sub = ChannelSubscriber('rt/lowstate', LowStateType)
        self.lowstate_sub.Init(self._lowstate_cb, 10)
        self.cmd_pub = ChannelPublisher('rt/lowcmd', LowCmdType); self.cmd_pub.Init()
        self.crc     = CRC()

    def _publish_zero_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = self.all_joint_names
        js.position = [0.0] * len(self.all_joint_names)
        self.joint_pub.publish(js)
        self.get_logger().info('Published zero joint states')

    def _on_init_timeout(self):
        if not self._received_joint and not self._init_done:
            self._publish_zero_joints()
            self._init_done = True
        self.destroy_timer(self._init_timer)

    def _lowstate_cb(self, msg: LowStateType):
        for i in ALL_JOINT_INDICES:
            self.current[i] = msg.motor_state[i].q
        if not hasattr(self, 'received') or not self.received:
            self.targets  = self.current.copy()
            self.smoothed = self.current.copy()
        if not hasattr(self, 'update_mode_machine_') or not self.update_mode_machine_:
            self.mode_machine_      = msg.mode_machine
            self.update_mode_machine_ = True
        self.received = True

    def _joint_state_cb(self, msg: JointState):
        if self.emergency_stop:
            return
        if not self._received_joint:
            self._received_joint = True
        for name, pos in zip(msg.name, msg.position):
            if name in self.name_to_q_index:
                self.q[self.name_to_q_index[name]] = pos

    def _ik_target_cb(self, msg: PoseStamped):
        if self.emergency_stop:
            return
        p = msg.pose.position
        o = msg.pose.orientation
        quat = pin.Quaternion(o.w, o.x, o.y, o.z)
        tgt  = SE3(quat.matrix(), np.array([p.x, p.y, p.z]))

        q_sol = self._solve_ik(tgt)
        if q_sol is None:
            return

        dq = q_sol - self.q_prev
        dq = np.clip(dq, -self.max_dq, self.max_dq)
        q_step = self.q_prev + dq

        self.q = (1 - self.alpha) * self.q_prev + self.alpha * q_step
        self.q_prev = self.q.copy()

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.all_joint_names
        js.position = [
            float(self.q[self.name_to_q_index[name]]) if name in self.right_joint_names else 0.0
            for name in self.all_joint_names
        ]
        self.joint_pub.publish(js)

    def _solve_ik(self, target: SE3, max_iter=50, tol=1e-4, damping=1e-6):
        q = self.q.copy()
        free_nv = sum(j.nv for j in self.model.joints if j.nq > 1)
        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            M_cur = self.data.oMf[self.eff_frame_id]
            err_tf = M_cur.inverse() * target
            err    = pin.log(err_tf).vector
            if np.linalg.norm(err) < tol:
                return q
            J6    = pin.computeFrameJacobian(
                self.model, self.data, q,
                self.eff_frame_id,
                pin.LOCAL_WORLD_ALIGNED
            )
            J_red = J6[:, free_nv:]
            JJt   = J_red @ J_red.T
            dq    = J_red.T @ np.linalg.solve(JJt + damping * np.eye(6), err)
            q[self.actuated_idx] += dq
        self.get_logger().warn("IK did not converge")
        return None

    def _on_estop(self):
        if not self.using_robot:
            self.emergency_stop = True
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
        self.emergency_stop = True


def main():
    rclpy.init()
    node = RightArmIKController()

    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = QWidget()
    window.setWindowTitle("Emergency Stop")
    layout = QVBoxLayout(window)
    btn = QPushButton("EMERCENY STOP")
    btn.setStyleSheet("font-size: 24px; background-color: red; color: white;")
    btn.clicked.connect(node._on_estop)
    layout.addWidget(btn)
    window.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
