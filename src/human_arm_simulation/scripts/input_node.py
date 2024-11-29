#!/usr/bin/python3

import rclpy
from rclpy.node import Node


class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
