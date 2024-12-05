#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import roboticstoolbox as rbt
from spatialmath import *
import numpy as np

from human_arm_interfaces.srv import *

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Service server
        self.create_service(MoveJ ,'/moveJ_q' ,self.moveJ_q_callback)
        self.create_service(MoveJ ,'/moveJ_target' ,self.moveJ_target_callback)
        self.create_service(MoveL ,'/moveL_target' ,self.moveL_target_callback)
        self.create_service(JointEffort ,'/input_wrench' ,self.input_wrench_callback)
        self.create_service(RobotReady ,'/robot_ready' ,self.robot_ready_callback)
        
        # Service client
        self.moveJ_client = self.create_client(MoveJ, '/moveJ')
        self.moveL_client = self.create_client(MoveL, '/moveL')
        self.calculation_effort_client = self.create_client(JointEffort, '/calculation_effort')

        # Defind DH-Table 3R-R-3R
        pi = np.pi
        self.human_arm = rbt.DHRobot([
                rbt.RevoluteMDH(alpha = pi/2), #joint 1
                rbt.RevoluteMDH(alpha = pi/2, offset = pi/2), #joint 2
                rbt.RevoluteMDH(alpha = -pi/2, offset = -pi/2), #joint 3
                rbt.RevoluteMDH(a = 0.4, alpha = -pi/2, offset = -pi/2), #joint 4
                rbt.RevoluteMDH(alpha = -pi/2, d = 0.4), #joint 5
                rbt.RevoluteMDH(alpha = pi/2, offset = pi/2), #joint 6
                rbt.RevoluteMDH(alpha = pi/2), #joint 7
            ],
            name = "Human Arm"
        )

        # Variable
        self.robot_ready = True
        self.target_in_workspace = False

        # moveJ variable
        self.q_goal = [0, 0, 0, 0, 0, 0, 0]

        # moveL variable
        self.target = [0, 0, 0, 0, 0, 0]
        self.velocity = 0.01

        # Input wrench variable
        self.wrench = [0, 0, 0, 0, 0, 0]

        # Display Node start
        self.get_logger().info(f'Controller Start Node.')

    def call_moveJ(self):
        msg = MoveJ.Request()
        msg.q1 = self.q_goal[0]
        msg.q2 = self.q_goal[1]
        msg.q3 = self.q_goal[2]
        msg.q4 = self.q_goal[3]
        msg.q5 = self.q_goal[4]
        msg.q6 = self.q_goal[5]
        msg.q7 = self.q_goal[6]

        self.moveJ_client.call_async(msg)

    def call_moveL(self):
        msg = MoveL.Request()
        msg.target.position.x = self.target[0]
        msg.target.position.y = self.target[1]
        msg.target.position.z = self.target[2]
        msg.target.orientation.x = self.target[3]
        msg.target.orientation.y = self.target[4]
        msg.target.orientation.z = self.target[5]
        
        msg.velocity = self.velocity

        self.moveL_client.call_async(msg)
    
    def call_calculation_effort(self):
        msg = JointEffort.Request()
        msg.wrench.force.x = self.wrench[0]
        msg.wrench.force.y = self.wrench[1]
        msg.wrench.force.z = self.wrench[2]
        msg.wrench.torque.x = self.wrench[3]
        msg.wrench.torque.y = self.wrench[4]
        msg.wrench.torque.z = self.wrench[5]
        
        self.calculation_effort_client.call_async(msg)

    def calculate_invert_kinematic(self):
        target_xyz = SE3(self.target[0], self.target[1], self.target[2])
        target_Rx = SE3.Rx(self.target[3])
        target_Ry = SE3.Ry(self.target[4])
        target_Rz = SE3.Rz(self.target[5])

        T_goal = target_xyz @ target_Rx @ target_Ry @ target_Rz
        q_sol = self.human_arm.ikine_LM(T_goal)

        self.target_in_workspace = q_sol.success
        
        if self.target_in_workspace:
            self.q_goal = q_sol.q

    def moveJ_q_callback(self, request:MoveJ.Request, response:MoveJ.Response):
        if self.robot_ready:
            self.q_goal[0] = request.q1
            self.q_goal[1] = request.q2
            self.q_goal[2] = request.q3
            self.q_goal[3] = request.q4
            self.q_goal[4] = request.q5
            self.q_goal[5] = request.q6
            self.q_goal[6] = request.q7

            self.call_moveJ()
            self.robot_ready = False
        else:
            self.get_logger().info(f'Not ready to move')

        response.success =True

        return response

    def moveJ_target_callback(self, request:MoveJ.Request, response:MoveJ.Response):
        if self.robot_ready:
            self.target[0] = request.target.position.x
            self.target[1] = request.target.position.y
            self.target[2] = request.target.position.z - 1.0
            self.target[3] = request.target.orientation.x
            self.target[4] = request.target.orientation.y
            self.target[5] = request.target.orientation.z

            self.calculate_invert_kinematic()

            if self.target_in_workspace:
                self.get_logger().info(f'Target is in work space.')
                self.call_moveL()
                self.robot_ready = False
                response.target_in_workspace = True
            else:
                self.get_logger().info(f'Target out of work space can not move!')
        else:
            self.get_logger().info(f'Not ready to move')
        
        response.success =True
        return response

    def moveL_target_callback(self, request:MoveL.Request, response:MoveL.Response):
        if self.robot_ready:
            self.target[0] = request.target.position.x
            self.target[1] = request.target.position.y
            self.target[2] = request.target.position.z - 1.0
            self.target[3] = request.target.orientation.x
            self.target[4] = request.target.orientation.y
            self.target[5] = request.target.orientation.z

            self.calculate_invert_kinematic()

            if self.target_in_workspace:
                self.get_logger().info(f'Target is in work space.')
                self.call_moveL()
                self.robot_ready = False
                response.target_in_workspace = True
            else:
                self.get_logger().info(f'Target out of work space can not move!')
        else:
            self.get_logger().info(f'Not ready to move')
        
        response.success =True
        return response

    def input_wrench_callback(self, request:JointEffort.Request, response:JointEffort.Response):
        if self.robot_ready:
            self.wrench[0] = request.wrench.force.x
            self.wrench[1] = request.wrench.force.y
            self.wrench[2] = request.wrench.force.z
            self.wrench[3] = request.wrench.torque.x
            self.wrench[4] = request.wrench.torque.y
            self.wrench[5] = request.wrench.torque.z

            self.robot_ready = False
            response.success = True
            self.call_calculation_effort()
        else:
            self.get_logger().info(f'Not ready to calculation joint effort.')
        return response
    
    def robot_ready_callback(self, request:RobotReady.Request, response:RobotReady.Response):
        self.robot_ready = request.ready
        self.get_logger().info(f'Ready to move')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
