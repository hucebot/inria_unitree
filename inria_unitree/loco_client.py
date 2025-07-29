#!/usr/bin/env python3

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)


def _rpc_get_int(client, api_id):
    try:
        code, data = client._Call(api_id, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


class G1LocoClient(Node):
    def __init__(self):
        super().__init__("g1_loco_client")
        ChannelFactoryInitialize(0, 'eth0')
        self.robot = LocoClient()
        self.robot.SetTimeout(10.0)
        self.robot.Init()
        self.robot.Damp()
        self.balanced = False
        self.robot_stopped = False

        self.current_id = self.get_fsm_id()
        self.current_mode = self.get_fsm_mode()
        print(f"Current FSM ID: {self.current_id}, Mode: {self.current_mode}")
        self.robot.SetFsmId(4)
        print(f"Set FSM ID to: {self.get_fsm_id()}")

        self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_callback,
            10
        )

        self.create_subscription(
            Bool,
            'start_balancing',
            self.start_balancing_callback,
            10
        )

        self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            10
        )

    def get_fsm_id(self):
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)

    def get_fsm_mode(self):
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_MODE)

    def emergency_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("EMERGENCY STOP ACTIVATED!")
            self.robot_stopped = True
            self.balanced = False
            self.robot.Damp()

    def start_balancing_callback(self, msg: Bool):
        if msg.data and not self.balanced:
            self.get_logger().info("Starting balancing procedure...")
            self.entering_balancing(max_height=0.5, step=0.02)
            self.get_logger().info("Balancing procedure completed.")
        elif self.balanced:
            self.get_logger().info("Already balanced, no action taken.")

    def joystick_callback(self, msg: Joy):
        if not self.balanced:
            self.get_logger().warn("Robot is not balanced, cannot move.")
            return

        if self.robot_stopped:
            self.get_logger().warn("Robot is stopped, cannot move.")
            return

        # button for emergency stop
        if msg.buttons[0] == 1:
            self.get_logger().warn("Emergency stop button pressed!")
            self.robot_stopped = True
            self.balanced = False
            self.robot.Damp()
            return
        
        # button for starting balancing
        if msg.buttons[1] == 1:
            if not self.balanced:
                self.get_logger().info("Starting balancing procedure...")
                self.entering_balancing(max_height=0.5, step=0.02)
                self.get_logger().info("Balancing procedure completed.")
            else:
                self.get_logger().info("Already balanced, no action taken.")
            return

        
        vx = msg.axes[1] * 0.5
        vy = msg.axes[0] * 0.5
        yaw = msg.axes[3] * 0.5

        if abs(vx) < 0.05 and abs(vy) < 0.05 and abs(yaw) < 0.05:
            self.robot.StopMove()
            return

        self.robot.Move(vx=vx, vy=vy, vyaw=yaw, continous_move=True)

    def entering_balancing(self, max_height=0.5, step=0.02):
        height = 0.0
        while height < max_height and not self.robot_stopped:
            height += step
            self.robot.SetStandHeight(height)
            if self.get_fsm_mode() == 0 and height >= 0.2:
                self.get_logger().info(f"Reached max height: {height}")
                break
            elif self.get_fsm_mode() != 0:
                self.get_logger().warn("Problems during balancing, stopping...")
                break

        self.robot.BalanceStand(0)
        self.robot.SetStandHeight(height)
        self.robot.Start()
        self.balanced = True

def main(args=None):
    try:
        rclpy.init(args=args)
        node = G1LocoClient()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        G1LocoClient.robot.Damp()
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()
