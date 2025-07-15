#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, Point, PointStamped

from rclpy.qos import QoSProfile

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


class G1JointIndex:
    LeftHipPitch     = 0
    LeftHipRoll      = 1
    LeftHipYaw       = 2
    LeftKnee         = 3
    LeftAnklePitch   = 4
    LeftAnkleRoll    = 5

    RightHipPitch    = 6
    RightHipRoll     = 7
    RightHipYaw      = 8
    RightKnee        = 9
    RightAnklePitch  = 10
    RightAnkleRoll   = 11

    WaistYaw         = 12
    WaistRoll        = 13
    WaistPitch       = 14

    LeftShoulderPitch  = 15
    LeftShoulderRoll   = 16
    LeftShoulderYaw    = 17
    LeftElbow          = 18
    LeftWristRoll      = 19
    LeftWristPitch     = 20
    LeftWristYaw       = 21

    RightShoulderPitch = 22
    RightShoulderRoll  = 23
    RightShoulderYaw   = 24
    RightElbow         = 25
    RightWristRoll     = 26
    RightWristPitch    = 27
    RightWristYaw      = 28

_joint_index_to_ros_name = {
    G1JointIndex.LeftHipPitch:      "left_hip_pitch_joint",
    G1JointIndex.LeftHipRoll:       "left_hip_roll_joint",
    G1JointIndex.LeftHipYaw:        "left_hip_yaw_joint",
    G1JointIndex.LeftKnee:          "left_knee_joint",
    G1JointIndex.LeftAnklePitch:    "left_ankle_pitch_joint",
    G1JointIndex.LeftAnkleRoll:     "left_ankle_roll_joint",
    G1JointIndex.RightHipPitch:     "right_hip_pitch_joint",
    G1JointIndex.RightHipRoll:      "right_hip_roll_joint",
    G1JointIndex.RightHipYaw:       "right_hip_yaw_joint",
    G1JointIndex.RightKnee:         "right_knee_joint",
    G1JointIndex.RightAnklePitch:   "right_ankle_pitch_joint",
    G1JointIndex.RightAnkleRoll:    "right_ankle_roll_joint",
    G1JointIndex.WaistYaw:          "waist_yaw_joint",
    G1JointIndex.WaistRoll:         "waist_roll_joint",
    G1JointIndex.WaistPitch:        "waist_pitch_joint",
    G1JointIndex.LeftShoulderPitch: "left_shoulder_pitch_joint",
    G1JointIndex.LeftShoulderRoll:  "left_shoulder_roll_joint",
    G1JointIndex.LeftShoulderYaw:   "left_shoulder_yaw_joint",
    G1JointIndex.LeftElbow:         "left_elbow_joint",
    G1JointIndex.LeftWristRoll:     "left_wrist_roll_joint",
    G1JointIndex.LeftWristPitch:    "left_wrist_pitch_joint",
    G1JointIndex.LeftWristYaw:      "left_wrist_yaw_joint",
    G1JointIndex.RightShoulderPitch:"right_shoulder_pitch_joint",
    G1JointIndex.RightShoulderRoll: "right_shoulder_roll_joint",
    G1JointIndex.RightShoulderYaw:  "right_shoulder_yaw_joint",
    G1JointIndex.RightElbow:        "right_elbow_joint",
    G1JointIndex.RightWristRoll:    "right_wrist_roll_joint",
    G1JointIndex.RightWristPitch:   "right_wrist_pitch_joint",
    G1JointIndex.RightWristYaw:     "right_wrist_yaw_joint",
}


class RosBridge(Node):
    def __init__(self):
        super().__init__('ros_lowstate_reader')

        self.declare_parameter('interface', 'eth0')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.get_logger().info(f'Using interface: {interface}')
        self.ns = ''

        ChannelFactoryInitialize(0, interface)

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, f"{self.ns}/joint_states", qos_profile)
        self.imu_publisher = self.create_publisher(Imu, f"{self.ns}/imu", qos_profile)
        self.odometry_pub = self.create_publisher(Odometry, f"{self.ns}/odometry", qos_profile)

        self.joint_indices = sorted(_joint_index_to_ros_name.keys())

        self.joint_names = [ _joint_index_to_ros_name[i] for i in self.joint_indices ]

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.callback_lowstate)

    def callback_lowstate(self, msg: LowState_):

        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.orientation.x = msg.imu_state.quaternion[0]
        imu_msg.orientation.y = msg.imu_state.quaternion[1]
        imu_msg.orientation.z = msg.imu_state.quaternion[2]
        imu_msg.orientation.w = msg.imu_state.quaternion[3]
        imu_msg.angular_velocity.x = msg.imu_state.gyroscope[0]
        imu_msg.angular_velocity.y = msg.imu_state.gyroscope[1]
        imu_msg.angular_velocity.z = msg.imu_state.gyroscope[2]
        imu_msg.linear_acceleration.x = msg.imu_state.accelerometer[0]
        imu_msg.linear_acceleration.y = msg.imu_state.accelerometer[1]
        imu_msg.linear_acceleration.z = msg.imu_state.accelerometer[2]
        self.imu_publisher.publish(imu_msg)

        posiciones = []
        for idx in self.joint_indices:
            if idx < len(msg.motor_state):
                posiciones.append(msg.motor_state[idx].q)
            else:

                self.get_logger().error(
                    f'The message LowState_ only has {len(msg.motor_state)} motors, '
                    f'but at least index {idx} was expected. Not publishing.'
                )
                return

        if len(posiciones) != len(self.joint_state_msg.name):
            self.get_logger().error(
                f'Total number of positions ({len(posiciones)}) does not match '
                f'number of joint names ({len(self.joint_state_msg.name)}). Not publishing.'
            )
            return

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = posiciones

        self.joint_pub.publish(self.joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
