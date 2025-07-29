#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import numpy as np

class Joystick(Node):
    def __init__(self):
        super().__init__("joystick_republisher")

        self.declare_parameter("joy_topic", "/inria_unitree/joy")
        self.declare_parameter("lin_scale", 0.2)
        self.declare_parameter("ang_scale", 1.0)

        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value
        self.lin_scale = self.get_parameter("lin_scale").get_parameter_value().double_value
        self.ang_scale = self.get_parameter("ang_scale").get_parameter_value().double_value

        self.publisher = self.create_publisher(Float32MultiArray, "/joystick/velocity", 10)
        self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.get_logger().info("Joystick republisher initialized")

    def joy_callback(self, msg: Joy):
        vel_x = msg.axes[1] * self.lin_scale
        vel_y = msg.axes[0] * self.lin_scale
        vel_ang = msg.axes[3] * self.ang_scale

        deadman_button = msg.buttons[7]

        if deadman_button == 1:
            ref = np.array([vel_x, vel_y, vel_ang], dtype=np.float32)
            msg_out = Float32MultiArray()
            msg_out.data = ref.tolist()
            self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = Joystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
