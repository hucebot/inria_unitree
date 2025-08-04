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

        self.using_robot = True
        self.robot_stopped = False
        self.balanced = False
        self.prev_buttons = {}

        if self.using_robot:
            ChannelFactoryInitialize(0, 'eth0')
            self.robot = LocoClient()
            self.robot.SetTimeout(10.0)
            self.robot.Init()
            self.robot.Damp()
            self.current_id = self.get_fsm_id()
            self.current_mode = self.get_fsm_mode()
            print(f"Current FSM ID: {self.current_id}, Mode: {self.current_mode}")

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
            '/g1pilot/joy',
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
        if not self.prev_buttons:
            self.prev_buttons = {i: 0 for i in range(len(msg.buttons))}

        if not self.balanced:
            self.get_logger().warn("Robot is not balanced, cannot move.")

        if self.robot_stopped:
            self.get_logger().warn("Robot is stopped, cannot move.")

        if msg.axes[-1] == -1.0:
            self.robot.SetFsmId(4)
            print(f"Set FSM ID to: {self.get_fsm_id()}")
            self.robot_stopped = False
            self.balanced = False
        
        if msg.buttons[0] == 1 and self.prev_buttons[0] == 0:
            self.robot.ShakeHand()

        # button for emergency stop - L1
        if msg.buttons[4] == 1:
            self.get_logger().warn("Emergency stop button pressed!")
            self.robot_stopped = True
            self.balanced = False
            self.robot.Damp()
        
        # button for starting balancing - R1
        if msg.buttons[5] == 1:
            print("R1 button pressed, starting balancing procedure...")
            if not self.balanced:
                self.get_logger().info("Starting balancing procedure...")
                self.entering_balancing(max_height=0.5, step=0.02)
                self.get_logger().info("Balancing procedure completed.")
            else:
                self.get_logger().info("Already balanced, no action taken.")

        # We only send commands if the dead man button is pressed (L2) and the robot is balanced and not stopped
        if msg.buttons[7] == 0 and not self.robot_stopped and self.balanced:
            self.robot.StopMove()

        if msg.buttons[7] == 1 and not self.robot_stopped and self.balanced:
            vx = round(msg.axes[1] * 0.2 * -1, 2)
            vy = round(msg.axes[0] * 0.2, 2)
            yaw = round(msg.axes[3] * 0.2, 2)

            self.get_logger().info(f"Moving with vx: {vx}, vy: {vy}, yaw: {yaw}")
            if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(yaw) < 0.03:
                self.robot.StopMove()
                return

            self.robot.Move(vx=vx, vy=vy, vyaw=yaw, continous_move=True)

        self.prev_buttons = {i: msg.buttons[i] for i in range(len(msg.buttons))}

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
