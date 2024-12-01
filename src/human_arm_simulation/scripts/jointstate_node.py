#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import numpy as np 

from sensor_msgs.msg import JointState
from human_arm_interfaces.srv import *


class JointstateNode(Node):
    def __init__(self):
        super().__init__('jointstate_node')

        # Pub Topic
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        # Service server

        # Service client

        # Timmer 
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)

        # Defind DH-Table
        self.robot = rtb.DHRobot([
                rtb.RevoluteMDH(d = 200), # joint 1
                rtb.RevoluteMDH(alpha = -pi/2 ,d = -120 ,offset = -pi/2), # joint 2
                rtb.RevoluteMDH(a = 250 ,d = 100 ,offset = pi/2), # joint 3
                rtb.RevoluteMDH(alpha = pi/2, d = 280), # End-effector
            ],
            name = "3R_Robot"
        )

        # Variable
        self.teleop_run = False
        self.teleop_ref = ''
        self.auto_to_target = False

def main(args=None):
    rclpy.init(args=args)
    node = JointstateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
