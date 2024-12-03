#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from human_arm_interfaces.srv import *
import pygame
from config.element import *


class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        # Service client
        self.moveJ_q_client = self.create_client(MoveJ, '/moveJ_q')
        self.moveJ_target_client = self.create_client(MoveJ, '/moveJ_target')
        self.moveL_target_client = self.create_client(MoveL, '/moveL_target')
        self.input_wrench_client = self.create_client(JointEffort, '/input_wrench')

        pygame.init()
        self.SCREEN_WIDTH = 800
        self.SCREEN_HEIGHT = 1000

        # Screen setup
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.display.set_caption("Human Arm Robot UI")

        # Colors
        self.WHITE = (255, 255, 255)

        # Header and button properties
        self.header_height = 100
        self.button_width = 200
        self.button_height = 60
        self.button_spacing = 40
        self.button_y = self.header_height + 20

        # Toggle buttons
        self.toggle_buttons = [
            ToggleButton(
                self.SCREEN_WIDTH // 2 - self.button_width - self.button_spacing // 2,
                self.button_y,
                self.button_width,
                self.button_height,
                "MoveJ",
                active=True
            ),
            ToggleButton(
                self.SCREEN_WIDTH // 2 + self.button_spacing // 2,
                self.button_y,
                self.button_width,
                self.button_height,
                "MoveL"
            )
        ]

        # MoveJ Titles
        self.section_y = self.header_height + 150
        self.movej_titles = [
            {"text": "Move by Q", "pos": (self.SCREEN_WIDTH // 4, self.section_y + self.button_height // 2)},
            {"text": "Target J Ref by World", "pos": (3 * self.SCREEN_WIDTH // 4, self.section_y + self.button_height // 2)}
        ]

        # Scrollbars
        self.left_section_top = self.section_y + self.button_height + 50
        self.scrollbar_width = 100
        self.scrollbar_height = 20
        self.scrollbar_x = 150
        self.scrollbar_spacing = 50

        joint_limits = [
            (-0.87, 3.14),
            (0, 1.57),
            (0, 3.14),
            (0, 2.16),
            (-1.57, 1.57),
            (-1.57, 1.04),
            (-0.34, 0.69)
        ]

        self.scrollbars = []
        y_pos = self.left_section_top
        for i, (lower, upper) in enumerate(joint_limits):
            self.scrollbars.append(ScrollBar(self.scrollbar_x, y_pos, self.scrollbar_width, self.scrollbar_height, f"q{i+1}", lower, upper))
            y_pos += self.scrollbar_spacing

        self.move_button = Button(self.scrollbar_x, y_pos + 20, 100, 50, "Move")

        # Input boxes for Target J Ref by World
        self.x_input = InputBox(self.SCREEN_WIDTH // 2 + 100, 300 + 50, 200, 40)
        self.y_input = InputBox(self.SCREEN_WIDTH // 2 + 100, 360 + 50, 200, 40)
        self.z_input = InputBox(self.SCREEN_WIDTH // 2 + 100, 420 + 50, 200, 40)

        self.roll_input = InputBox(self.SCREEN_WIDTH // 2 + 100, 500 + 50, 200, 40)
        self.pitch_input = InputBox(self.SCREEN_WIDTH // 2 + 100, 560 + 50, 200, 40)
        self.yaw_input = InputBox(self.SCREEN_WIDTH // 2 + 100, 620 + 50, 200, 40)

        self.target_move_button = Button(self.SCREEN_WIDTH // 2 + 150, 700 + 50, 100, 50, "Move")

        # Input boxes for MoveL - Position
        self.position_x_input = InputBox(self.SCREEN_WIDTH // 6 - 50, 250 + 100, 100, 40, "")
        self.position_y_input = InputBox(self.SCREEN_WIDTH // 6 - 50, 310 + 100, 100, 40, "")
        self.position_z_input = InputBox(self.SCREEN_WIDTH // 6 - 50, 370 + 100, 100, 40, "")

        self.position_roll_input = InputBox(self.SCREEN_WIDTH // 6 - 50, 430 + 120, 100, 40, "")
        self.position_pitch_input = InputBox(self.SCREEN_WIDTH // 6 - 50, 490 + 120, 100, 40, "")
        self.position_yaw_input = InputBox(self.SCREEN_WIDTH // 6 - 50, 550 + 120, 100, 40, "")

        self.movel_move_button = Button(self.SCREEN_WIDTH // 6 - 50, 650 + 120, 100, 50, "Move")

        # Input boxes for Wrench - Force
        self.force_x_input = InputBox(self.SCREEN_WIDTH // 2 - 20, 250 + 100, 100, 40, "")
        self.force_y_input = InputBox(self.SCREEN_WIDTH // 2 - 20, 310 + 100, 100, 40, "")
        self.force_z_input = InputBox(self.SCREEN_WIDTH // 2 - 20, 370 + 100, 100, 40, "")

        self.torque_x_input = InputBox(self.SCREEN_WIDTH // 2 - 20, 450 + 100, 100, 40, "")
        self.torque_y_input = InputBox(self.SCREEN_WIDTH // 2 - 20, 510 + 100, 100, 40, "")
        self.torque_z_input = InputBox(self.SCREEN_WIDTH // 2 - 20, 570 + 100, 100, 40, "")

        self.calculate_button = Button(self.SCREEN_WIDTH // 2 + 50, 650 + 120, 150, 50, "Calculate")

        # Placeholder for joint efforts
        self.joint_effort_values = ["Joint 1: 0.00", "Joint 2: 0.00", "Joint 3: 0.00",
                                    "Joint 4: 0.00", "Joint 5: 0.00", "Joint 6: 0.00", "Joint 7: 0.00"]

        self.show_values = False
        self.running = True



def main(args=None):
    rclpy.init(args=args)
    pygame.init()
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__=='__main__':
    main()
