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

# --- Joint mapping definitions ---
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

class RightArmIKController(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'right_arm_ik_controller')
        QWidget.__init__(self)
        self.setWindowTitle("Right Arm IK Only (No Robot Command)")

        self.alpha = 0.2
        self.max_dq = 0.2
        self.emergency_stop = False

        self.joints = np.zeros(len(ALL_JOINT_INDICES))
        self.joints_prev = self.joints.copy()

        qos = QoSProfile(depth=10)
        self.ik_pub = self.create_publisher(JointState, '/ik_joint_states', qos)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, qos)
        self.create_subscription(PoseStamped, '/right_hand_goal', self._ik_target_cb, qos)


        pkg_share = get_package_share_directory('g1_description')
        urdf = os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof.urdf')
        mesh = os.path.join(pkg_share, 'description_files', 'meshes')
        self.model, _, _ = pin.buildModelsFromUrdf(
            urdf, package_dirs=[mesh]
        )
        self.data = pin.Data(self.model)

        self.name_to_index = _joint_index_to_ros_name.copy()
        self.ros_names = [self.name_to_index[i] for i in ALL_JOINT_INDICES]

        self.name_to_q_index = {}
        for j in range(1, self.model.njoints):
            if self.model.joints[j].nq == 1:
                nm = self.model.names[j]
                if nm in self.ros_names:
                    self.name_to_q_index[nm] = self.model.joints[j].idx_q

        self.eff_frame = 'right_wrist_pitch_link'
        self.eff_frame_id = self.model.getFrameId(self.eff_frame)

        layout = QVBoxLayout(self)
        estop_btn = QPushButton("EMERGENCY STOP")
        estop_btn.setStyleSheet("font-size: 24px; background-color: red; color: white;")
        estop_btn.clicked.connect(self._on_estop)
        layout.addWidget(estop_btn)

        self.received_joint = False
        self._qt_spin = QtCore.QTimer(self)
        self._qt_spin.timeout.connect(lambda: rclpy.spin_once(self, timeout_sec=0))
        self._qt_spin.start(10)

    def _joint_state_cb(self, msg: JointState):
        if self.emergency_stop:
            return
        self.received_joint = True
        name_idx = {n: i for i, n in enumerate(self.ros_names)}
        for name, pos in zip(msg.name, msg.position):
            if name in name_idx:
                self.joints[name_idx[name]] = pos

    def _ik_target_cb(self, msg: PoseStamped):
        if self.emergency_stop or not self.received_joint:
            return

        q = pin.neutral(self.model)
        for i, name in enumerate(self.ros_names):
            if name in self.name_to_q_index:
                q[self.name_to_q_index[name]] = self.joints[i]

        p, o = msg.pose.position, msg.pose.orientation
        quat = pin.Quaternion(o.w, o.x, o.y, o.z)
        target = SE3(quat.matrix(), np.array([p.x, p.y, p.z]))

        q_sol = self._solve_ik(q, target)
        if q_sol is None:
            return
        joints_sol = np.zeros(len(self.ros_names))
        for i, name in enumerate(self.ros_names):
            if name in self.name_to_q_index:
                joints_sol[i] = q_sol[self.name_to_q_index[name]]

        dq = np.clip(joints_sol - self.joints_prev, -self.max_dq, self.max_dq)
        joints_step = self.joints_prev + dq
        self.joints = (1 - self.alpha) * self.joints_prev + self.alpha * joints_step
        self.joints_prev = self.joints.copy()

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.ros_names
        js.position = [float(x) for x in self.joints]
        self.ik_pub.publish(js)

    def _solve_ik(self, q_init, target: SE3, max_iter=50, tol=1e-4, damping=1e-6):
        q = q_init.copy()
        free_nv = sum(j.nv for j in self.model.joints if j.nq > 1)
        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            M_cur = self.data.oMf[self.eff_frame_id]
            err_tf = M_cur.inverse() * target
            err = pin.log(err_tf).vector
            if np.linalg.norm(err) < tol:
                return q
            J6 = pin.computeFrameJacobian(
                self.model, self.data, q, self.eff_frame_id, pin.LOCAL_WORLD_ALIGNED
            )
            J_red = J6[:, free_nv:]
            JJt = J_red @ J_red.T
            dq = J_red.T @ np.linalg.solve(JJt + damping * np.eye(6), err)

            actuated_indices = [self.name_to_q_index[name] for name in self.ros_names]
            q[actuated_indices] += dq[:len(actuated_indices)]
        self.get_logger().warn("IK did not converge")
        return None

    def _on_estop(self):
        self.emergency_stop = True
        self.get_logger().error("Emergency stop triggered")

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
