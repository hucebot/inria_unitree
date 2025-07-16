#!/usr/bin/env python3

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

import pinocchio as pin
import meshcat
from pinocchio.visualize import MeshcatVisualizer
from ament_index_python.packages import get_package_share_directory

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

        self.using_robot = self.declare_parameter('use_robot', False).value
        self.iface = self.declare_parameter('interface', 'eth0').value
        qos = QoSProfile(depth=10)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)
        self._received_joint = False
        self._init_done = False

        if self.using_robot:
            self._init_unitree()

        self.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, qos)

        pkg_share = get_package_share_directory('g1_description')
        urdf = os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof.urdf')
        meshes = os.path.join(pkg_share, 'description_files', 'meshes')

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf,
            package_dirs=[meshes],
            root_joint=pin.JointModelFreeFlyer()
        )

        self.q = pin.neutral(self.model)
        self.name_to_q_index = {}
        for j in range(1, self.model.njoints):
            joint = self.model.joints[j]
            if joint.nq == 1: 
                pin_name = self.model.names[j]
                self.name_to_q_index[pin_name] = joint.idx_q

        self._init_visualization()


        self._init_timer = self.create_timer(5.0, self._on_init_timeout)

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

    def _init_visualization(self):
        self.meshcat_client = meshcat.Visualizer()
        self.meshcat_vis = MeshcatVisualizer(
            self.model, self.collision_model, self.visual_model,
            self.meshcat_client
        )
        self.meshcat_vis.initViewer(open=True)
        self.meshcat_vis.loadViewerModel()
        self.meshcat_vis.displayVisuals(True)
        self.get_logger().info('Meshcat ready at http://127.0.0.1:7000/')

    def _publish_zero_joints(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        names = [name for _, name in sorted(
            _joint_index_to_ros_name.items(), key=lambda x: x[0]
        )]
        js.name = names
        js.position = [0.0] * len(names)
        self.joint_pub.publish(js)
        self.get_logger().info('Published initial zero joint positions')

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

        # Render in Meshcat
        self.meshcat_vis.display(self.q)


def main(args=None):
    rclpy.init(args=args)
    node = RightArmIKController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
