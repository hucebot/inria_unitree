#!/usr/bin/env python3

import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import pinocchio as pin
from pinocchio import SE3
from pinocchio.visualize import MeshcatVisualizer

import meshcat
from ament_index_python.packages import get_package_share_directory

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_ as LowCmdMessage
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdType, LowState_ as LowStateType
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

_joint_index_to_ros_name = {
    0: "left_hip_pitch_joint",   1: "left_hip_roll_joint",    2: "left_hip_yaw_joint",
    3: "left_knee_joint",        4: "left_ankle_pitch_joint", 5: "left_ankle_roll_joint",
    6: "right_hip_pitch_joint",  7: "right_hip_roll_joint",   8: "right_hip_yaw_joint",
    9: "right_knee_joint",      10: "right_ankle_pitch_joint",11: "right_ankle_roll_joint",
   12: "waist_yaw_joint",       13: "waist_roll_joint",      14: "waist_pitch_joint",
   15: "left_shoulder_pitch_joint", 16: "left_shoulder_roll_joint", 17: "left_shoulder_yaw_joint",
   18: "left_elbow_joint",      19: "left_wrist_roll_joint", 20: "left_wrist_pitch_joint",
   21: "left_wrist_yaw_joint",  22: "right_shoulder_pitch_joint",23: "right_shoulder_roll_joint",
   24: "right_shoulder_yaw_joint",25: "right_elbow_joint",    26: "right_wrist_roll_joint",
   27: "right_wrist_pitch_joint",28: "right_wrist_yaw_joint",
}
ALL_JOINT_INDICES = list(range(29))


class RightArmIKController(Node):
    def __init__(self):
        super().__init__('right_arm_ik_controller')
        qos = QoSProfile(depth=10)

        self.using_robot = self.declare_parameter('use_robot', False).value
        self.iface       = self.declare_parameter('interface', 'eth0').value

        self.joint_pub      = self.create_publisher(JointState, '/joint_states', qos)
        self._received_joint = False
        self._init_done      = False

        if self.using_robot:
            self._init_unitree()

        self.create_subscription(JointState,  '/joint_states', self._joint_state_cb, qos)
        self.create_subscription(PoseStamped, '/right_hand_goal',     self._ik_target_cb, qos)

        pkg_share = get_package_share_directory('g1_description')
        urdf_path = os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof.urdf')
        mesh_dir  = os.path.join(pkg_share, 'description_files', 'meshes')

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path,
            package_dirs=[mesh_dir],
            root_joint=pin.JointModelFreeFlyer()
        )
        self.data = pin.Data(self.model)

        for j in range(self.model.njoints):
            joint = self.model.joints[j]
            name  = self.model.names[j]
            self.get_logger().info(f"  [{j:2d}] name='{name}', nq={joint.nq}, idx_q={joint.idx_q}")

        self.q = pin.neutral(self.model)
        self.name_to_q_index = {}
        for j in range(1, self.model.njoints):
            joint = self.model.joints[j]
            if joint.nq == 1 and self.model.names[j] in _joint_index_to_ros_name.values():
                self.name_to_q_index[self.model.names[j]] = joint.idx_q

        self.actuated_idx = sorted(self.name_to_q_index.values())
        self.get_logger().info(f"Actuated idx (q): {self.actuated_idx}")

        self.eff_frame    = 'right_wrist_yaw_joint'
        self.eff_frame_id = self.model.getFrameId(self.eff_frame)

        self._init_visualization()

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


    def _init_visualization(self):
        self.meshcat_client = meshcat.Visualizer()
        self.meshcat_vis    = MeshcatVisualizer(
            self.model, self.collision_model, self.visual_model,
            self.meshcat_client
        )
        self.meshcat_vis.initViewer(open=True)
        self.meshcat_vis.loadViewerModel()
        self.meshcat_vis.displayVisuals(True)
        self.get_logger().info('Meshcat visualization initialized - http://localhost:7001/static/')


    def _publish_zero_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        names = [n for _, n in sorted(_joint_index_to_ros_name.items(), key=lambda x: x[0])]
        js.name     = names
        js.position = [0.0] * len(names)
        self.joint_pub.publish(js)
        self.get_logger().info('Published zero joint states')

    def _on_init_timeout(self):
        if not self._received_joint and not self._init_done:
            self._publish_zero_joints()
            self._init_done = True
        self.destroy_timer(self._init_timer)


    def _lowstate_cb(self, msg: LowStateType):
        pass


    def _joint_state_cb(self, msg: JointState):
        if not self._received_joint:
            self._received_joint = True

        for name, pos in zip(msg.name, msg.position):
            if name in self.name_to_q_index:
                self.q[self.name_to_q_index[name]] = pos

        self.meshcat_vis.display(self.q)


    def _ik_target_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        quat = pin.Quaternion(o.w, o.x, o.y, o.z)
        tgt = SE3(quat.matrix(), np.array([p.x, p.y, p.z]))

        q_sol = self._solve_ik(tgt)

        self.q = q_sol
        self.meshcat_vis.display(self.q)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        names = [n for _, n in sorted(_joint_index_to_ros_name.items(), key=lambda x: x[0])]
        js.name     = names
        js.position = [float(self.q[idx]) for idx in self.actuated_idx]
        self.joint_pub.publish(js)


    def _solve_ik(self, target: SE3, max_iter=50, tol=1e-4, damping=1e-6) -> np.ndarray:
        q = self.q.copy()

        free_nv = sum(joint.nv for joint in self.model.joints if joint.nq > 1)

        for _ in range(max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            M_cur = self.data.oMf[self.eff_frame_id]
            err_tf = M_cur.inverse() * target
            err    = pin.log(err_tf).vector
            if np.linalg.norm(err) < tol:
                break

            J6 = pin.computeFrameJacobian(
                self.model, self.data, q,
                self.eff_frame_id,
                pin.LOCAL_WORLD_ALIGNED
            )

            J_red = J6[:, free_nv:]

            JJt = J_red @ J_red.T
            dq  = J_red.T @ np.linalg.solve(JJt + damping * np.eye(6), err)

            q[self.actuated_idx] += dq

        return q


def main(args=None):
    rclpy.init(args=args)
    node = RightArmIKController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
