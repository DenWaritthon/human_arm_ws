#!/usr/bin/python3

import rclpy
from rclpy.node import Node

import roboticstoolbox as rbt
from spatialmath import *
import numpy as np
from scipy.linalg import svd

from sensor_msgs.msg import JointState
from human_arm_interfaces.srv import *


class JointstateNode(Node):
    def __init__(self):
        super().__init__('jointstate_node')

        # Pub Topic
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_effort_pub = self.create_publisher(JointState, "/joint_effort", 10)

        # Service server
        self.create_service(MoveJ ,'/moveJ' ,self.moveJ_callback)
        self.create_service(MoveL ,'/moveL' ,self.moveL_callback)
        self.create_service(JointEffort ,'/calculation_effort' ,self.calculation_effort_callback)

        # Service client
        self.robot_ready_client = self.create_client(RobotReady, '/robot_ready')

        # Timmer 
        self.dt = 0.01
        self.create_timer(self.dt, self.sim_loop)

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

        # joints publisher variable
        self.name = ["q1", "q2", "q3", "q4", "q5", "q6", "q7"] 
        self.q = [0, 0, 0, pi/12, 0, 0, 0]

        # moveJ variable
        self.moveJ = False
        self.q_goal = [0, 0, 0, 0, 0, 0, 0]
        self.kp = 1.5
        
        # moveL variable
        self.moveL = False
        self.target = [0, 0, 0, 0, 0, 0]
        self.velocity = 0.1
        self.vel = [0, 0, 0, 0, 0, 0]

        # joint effort variable
        self.wrench = [0, 0, 0, 0, 0, 0]
        self.joint_effort = [0, 0, 0, 0, 0, 0, 0]

        # Display Node start
        self.get_logger().info(f'JointState Start Node.')

    def send_robot_ready(self):
        msg = RobotReady.Request()
        msg.ready = True
        self.robot_ready_client.call_async(msg)

    def moveJ_callback(self, request:MoveJ.Request, response:MoveJ.Response):
        # get q value
        self.q_goal[0] = request.q1
        self.q_goal[1] = request.q2
        self.q_goal[2] = request.q3
        self.q_goal[3] = request.q4
        self.q_goal[4] = request.q5
        self.q_goal[5] = request.q6
        self.q_goal[6] = request.q7
        
        response.success = True

        self.moveJ = True
        self.get_logger().info(f'MoveJ start.')
        return response

    def moveL_callback(self, request:MoveL.Request, response:MoveL.Response):
        # get target
        self.target[0] = request.target.position.x
        self.target[1] = request.target.position.y
        self.target[2] = request.target.position.z
        self.target[3] = request.target.orientation.x
        self.target[4] = request.target.orientation.y
        self.target[5] = request.target.orientation.z

        # get velocity
        self.velocity = request.velocity

        response.success = True

        self.moveL = True
        self.get_logger().info(f'MoveL start.')
        return response
    
    def calculation_effort_callback(self, request:JointEffort.Request, response:JointEffort.Response):
        self.wrench[0] = request.wrench.force.x
        self.wrench[1] = request.wrench.force.y
        self.wrench[2] = request.wrench.force.z
        self.wrench[3] = request.wrench.torque.x
        self.wrench[4] = request.wrench.torque.y
        self.wrench[5] = request.wrench.torque.z
        
        J = self.human_arm.jacob0(self.q)
        self.joint_effort = np.dot(J.transpose(),self.wrench)

        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(self.q)): 
            msg.effort.append(self.joint_effort[i])
            msg.name.append(self.name[i])

        self.joint_effort_pub.publish(msg)
        self.send_robot_ready()

        response.success = True

        # self.get_logger().info(f'Joint effort is {self.joint_effort}')
        return response

    def velocity_calculation(self):
        current_pos = self.human_arm.fkine(self.q)
        
        target_direction = [
            self.target[0] - current_pos.x,
            self.target[1] - current_pos.y,
            self.target[2] - current_pos.z,
            self.target[3] - current_pos.rpy()[0], # roll
            self.target[4] - current_pos.rpy()[1], # pitch
            self.target[5] - current_pos.rpy()[2] # yall
        ]

        linear_size = np.sqrt(target_direction[0]**2 + target_direction[1]**2 + target_direction[2]**2)
        angular_size = np.sqrt(target_direction[3]**2 + target_direction[4]**2 + target_direction[5]**2)
        
        unit_target_direction =[
            target_direction[0] / linear_size,
            target_direction[1] / linear_size,
            target_direction[2] / linear_size,
            target_direction[3] / angular_size,
            target_direction[4] / angular_size,
            target_direction[5] / angular_size,
        ]
        
        self.vel = [
            unit_target_direction[0] * self.velocity,
            unit_target_direction[1] * self.velocity,
            unit_target_direction[2] * self.velocity,
            unit_target_direction[3] * self.velocity,
            unit_target_direction[4] * self.velocity,
            unit_target_direction[5] * self.velocity,
        ]

    def sim_loop(self):
        # Set header pub message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        if self.moveJ:
            dq = [0, 0, 0.0, 0.0, 0, 0.0, 0]
            
            for i in range(len(self.q)): 
                dq[i] = self.q_goal[i] - self.q[i]
            
            for i in range(len(self.q)):
                self.q[i] = self.q[i] + (dq[i] * self.dt * self.kp)

            if all(abs(i) < 0.01 for i in dq):
                self.moveJ = False
                self.get_logger().info(f'MoveJ stop.')
                self.send_robot_ready()

        if self.moveL:
            self.velocity_calculation()
            
            # calculation pseudo inverse jacobian 
            J = self.human_arm.jacob0(self.q)
            Jt = J.transpose()
            Jit = np.linalg.inv(np.dot(J,J.transpose()))
            J_new = np.dot(Jt,Jit)

            # Compute singular values
            U, S, Vh = svd(J)

            if np.any(S < 1e-6):
                self.moveL = False
                self.get_logger().info(f'MoveL stop.')
                self.get_logger().info(f'Human arm is near singularity.')
                self.send_robot_ready()
            else:
                # calculation q_dot
                q_dot = np.dot(J_new,self.vel)

                for i in range(len(self.q)):
                    self.q[i] = self.q[i] + q_dot[i]

                current_pos = self.human_arm.fkine(self.q)
                pos_check = [
                    self.target[0] - current_pos.x,
                    self.target[1] - current_pos.y,
                    self.target[2] - current_pos.z,
                    self.target[3] - current_pos.rpy()[0], # roll
                    self.target[4] - current_pos.rpy()[1], # pitch
                    self.target[5] - current_pos.rpy()[2] # yall
                ]
                
                if all(abs(i) < self.velocity for i in pos_check):
                    self.moveL = False
                    self.get_logger().info(f'MoveL stop.')
                    self.send_robot_ready()

        for i in range(len(self.q)): 
            joint_msg.position.append(self.q[i])
            joint_msg.name.append(self.name[i])

        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointstateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
