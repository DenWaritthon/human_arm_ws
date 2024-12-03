#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from human_arm_interfaces.srv import *


class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        # Service client
        self.moveJ_q_client = self.create_client(MoveJ, '/moveJ_q')
        self.moveJ_target_client = self.create_client(MoveJ, '/moveJ_target')
        self.moveL_target_client = self.create_client(MoveL, '/moveL_target')
        self.input_wrench_client = self.create_client(JointEffort, '/input_wrench')

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
