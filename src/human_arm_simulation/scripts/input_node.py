#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from human_arm_interfaces.srv import *
import pygame
from human_arm_simulation.element import *

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')

        # Service client
        self.moveJ_q_client = self.create_client(MoveJ, '/moveJ_q')
        self.moveJ_target_client = self.create_client(MoveJ, '/moveJ_target')
        self.moveL_target_client = self.create_client(MoveL, '/moveL_target')
        self.input_wrench_client = self.create_client(JointEffort, '/input_wrench')
        self.timer = self.create_timer(0.016, self.timer_callback)  # ~60Hz refresh rate

        pygame.init()
        self.SCREEN_WIDTH = 800
        self.SCREEN_HEIGHT = 1000

        # Screen setup
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.display.set_caption("Human Arm Robot UI")

        # Colors
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.GRAY = (200, 200, 200)
        self.BLUE = (0, 122, 255)
        self.ORANGE = (255, 165, 0)

        # Fonts
        pygame.font.init()
        self.FONT = pygame.font.SysFont('Arial', 30)
        self.SMALL_FONT = pygame.font.SysFont('Arial', 20)
        self.MED_FONT = pygame.font.SysFont('Arial',30)

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

        

    def timer_callback(self):
        if not self.running:
            return

        self.screen.fill(self.WHITE)

        # Draw Header
        title_surface = FONT.render("Human Arm Robot", True, self.BLACK)
        title_rect = title_surface.get_rect(center=(self.SCREEN_WIDTH // 2, self.header_height // 2))
        self.screen.blit(title_surface, title_rect)

        # Draw Top-Level Toggle Buttons
        for button in self.toggle_buttons:
            button.draw(self.screen)

        if self.toggle_buttons[0].active:  # MoveJ Mode
            # Draw vertical line to split into 2 sections
            pygame.draw.line(self.screen, self.BLACK, (self.SCREEN_WIDTH // 2, self.header_height + 100), (self.SCREEN_WIDTH // 2, self.SCREEN_HEIGHT), 3)

            # Draw MoveJ Titles
            for title in self.movej_titles:
                title_surface = FONT.render(title["text"], True, self.BLACK)
                title_rect = title_surface.get_rect(center=title["pos"])
                self.screen.blit(title_surface, title_rect)

            # ---- Move by Q Section ----
            # Draw scrollbars
            for scrollbar in self.scrollbars:
                scrollbar.draw(self.screen)

            # Draw the Move button for Move by Q
            self.move_button.draw(self.screen)

            # If the Move button was clicked, display the values
            if self.show_values:
                # Retrieve and display joint values from scrollbars
                values = [f"{scrollbar.label}: {scrollbar.get_value():.2f}" for scrollbar in self.scrollbars]
                for i, value in enumerate(values):
                    value_surface = SMALL_FONT.render(value, True, self.BLACK)
                    value_rect = value_surface.get_rect(topleft=(self.move_button.rect.left, self.move_button.rect.bottom + 10 + i * 25))
                    self.screen.blit(value_surface, value_rect)

            # ---- Target J Ref by World Section ----
            # Draw input fields for X, Y, Z
            self.x_input.draw(self.screen)
            self.y_input.draw(self.screen)
            self.z_input.draw(self.screen)

            # Draw input fields for Roll, Pitch, Yaw
            self.roll_input.draw(self.screen)
            self.pitch_input.draw(self.screen)
            self.yaw_input.draw(self.screen)

            # Draw the Move button for Target J Ref by World
            self.target_move_button.draw(self.screen)

            # Draw labels for X, Y, Z
            x_label = SMALL_FONT.render("X:", True, self.BLACK)
            self.screen.blit(x_label, (self.x_input.rect.x - 30, self.x_input.rect.y + 10))

            y_label = SMALL_FONT.render("Y:", True, self.BLACK)
            self.screen.blit(y_label, (self.y_input.rect.x - 30, self.y_input.rect.y + 10))

            z_label = SMALL_FONT.render("Z:", True, self.BLACK)
            self.screen.blit(z_label, (self.z_input.rect.x - 30, self.z_input.rect.y + 10))

            # Draw labels for Roll, Pitch, Yaw
            roll_label = SMALL_FONT.render("Roll:", True, self.BLACK)
            self.screen.blit(roll_label, (self.roll_input.rect.x - 60, self.roll_input.rect.y + 10))

            pitch_label = SMALL_FONT.render("Pitch:", True, self.BLACK)
            self.screen.blit(pitch_label, (self.pitch_input.rect.x - 60, self.pitch_input.rect.y + 10))

            yaw_label = SMALL_FONT.render("Yaw:", True, self.BLACK)
            self.screen.blit(yaw_label, (self.yaw_input.rect.x - 60, self.yaw_input.rect.y + 10))

        elif self.toggle_buttons[1].active:  # MoveL Mode
            # Draw dividing lines for 3 sections
            pygame.draw.line(self.screen, self.BLACK, (self.SCREEN_WIDTH // 3, self.header_height + 100), (self.SCREEN_WIDTH // 3, self.SCREEN_HEIGHT), 3)

            # Draw titles for each section
            position_title = FONT.render("Position", True, self.BLACK)
            position_title_rect = position_title.get_rect(center=(self.SCREEN_WIDTH // 6, self.header_height + 150))
            self.screen.blit(position_title, position_title_rect)

            Wrench_title = FONT.render("Wrench", True, self.BLACK)
            Wrench_title_rect = Wrench_title.get_rect(center=(self.SCREEN_WIDTH // 2, self.header_height + 150))
            self.screen.blit(Wrench_title, Wrench_title_rect)

            joint_effort_title = FONT.render("Joint Effort", True, self.BLACK)
            joint_effort_title_rect = joint_effort_title.get_rect(center=(5 * self.SCREEN_WIDTH // 6, self.header_height + 150))
            self.screen.blit(joint_effort_title, joint_effort_title_rect)

            # ---- 1st Section: Position ----
            # Draw input fields for Position X, Y, Z
            self.position_x_input.draw(self.screen)
            self.position_y_input.draw(self.screen)
            self.position_z_input.draw(self.screen)

            self.position_roll_input.draw(self.screen)
            self.position_pitch_input.draw(self.screen)
            self.position_yaw_input.draw(self.screen)

            pos_x_label = SMALL_FONT.render("X:", True, self.BLACK)
            self.screen.blit(pos_x_label, (self.position_x_input.rect.x - 30, self.position_x_input.rect.y + 10))

            pos_y_label = SMALL_FONT.render("Y:", True, self.BLACK)
            self.screen.blit(pos_y_label, (self.position_y_input.rect.x - 30, self.position_y_input.rect.y + 10))

            pos_z_label = SMALL_FONT.render("Z:", True, self.BLACK)
            self.screen.blit(pos_z_label, (self.position_z_input.rect.x - 30, self.position_z_input.rect.y + 10))

            pos_roll_label = SMALL_FONT.render("Roll:", True, self.BLACK)
            self.screen.blit(pos_roll_label, (self.position_roll_input.rect.x - 60, self.position_roll_input.rect.y + 10))

            pos_pitch_label = SMALL_FONT.render("Pitch:", True, self.BLACK)
            self.screen.blit(pos_pitch_label, (self.position_pitch_input.rect.x - 60, self.position_pitch_input.rect.y + 10))

            pos_yaw_label = SMALL_FONT.render("Yaw:", True, self.BLACK)
            self.screen.blit(pos_yaw_label, (self.position_yaw_input.rect.x - 60, self.position_yaw_input.rect.y + 10))

            # ---- 2nd Section: Force ----
            # Draw input fields for Force X, Y, Z
            self.force_x_input.draw(self.screen)
            self.force_y_input.draw(self.screen)
            self.force_z_input.draw(self.screen)
            self.torque_x_input.draw(self.screen)
            self.torque_y_input.draw(self.screen)
            self.torque_z_input.draw(self.screen)

            force_x_label = SMALL_FONT.render("Force X:", True, self.BLACK)
            self.screen.blit(force_x_label, (self.force_x_input.rect.x - 90, self.force_x_input.rect.y + 10))

            force_y_label = SMALL_FONT.render("Force Y:", True, self.BLACK)
            self.screen.blit(force_y_label, (self.force_y_input.rect.x - 90, self.force_y_input.rect.y + 10))

            force_z_label = SMALL_FONT.render("Force Z:", True, self.BLACK)
            self.screen.blit(force_z_label, (self.force_z_input.rect.x - 90, self.force_z_input.rect.y + 10))

            torque_x_label = SMALL_FONT.render("Torque X:", True, self.BLACK)
            self.screen.blit(torque_x_label, (self.torque_x_input.rect.x - 90, self.torque_x_input.rect.y + 10))

            torque_y_label = SMALL_FONT.render("Torque Y:", True, self.BLACK)
            self.screen.blit(torque_y_label, (self.torque_y_input.rect.x - 90, self.torque_y_input.rect.y + 10))

            torque_z_label = SMALL_FONT.render("Torque Z:", True, self.BLACK)
            self.screen.blit(torque_z_label, (self.torque_z_input.rect.x - 90, self.torque_z_input.rect.y + 10))

            # ---- 3rd Section: Joint Effort ----
            # Display text values for Joint Efforts
            for i, effort in enumerate(self.joint_effort_values):
                effort_surface = MED_FONT.render(effort, True, self.BLACK)
                effort_rect = effort_surface.get_rect(topleft=(5 * self.SCREEN_WIDTH // 6 - 90, 250 + 100 + i * 50))
                self.screen.blit(effort_surface, effort_rect)

            # Draw the Move button for MoveL
            self.movel_move_button.draw(self.screen)
            self.calculate_button.draw(self.screen)

        # Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                rclpy.shutdown()
                return

            # Handle toggle buttons
            for button in self.toggle_buttons:
                if button.handle_event(event):
                    for other_button in self.toggle_buttons:
                        other_button.active = False
                    button.active = True

            # Handle MoveJ events
            if self.toggle_buttons[0].active:  # MoveJ Mode
                # Handle scrollbars in "Move by Q"
                for scrollbar in self.scrollbars:
                    scrollbar.handle_event(event)

                if self.move_button.handle_event(event):  # "Move by Q" Move button
                    self.show_values = True
                    values = [f"{scrollbar.label}: {scrollbar.get_value():.2f}" for scrollbar in self.scrollbars]
                    print("Move by Q Joint Values:")
                    for value in values:
                        print(value)

                # Handle inputs in "Target J Ref by World"
                self.x_input.handle_event(event)
                self.y_input.handle_event(event)
                self.z_input.handle_event(event)
                self.roll_input.handle_event(event)
                self.pitch_input.handle_event(event)
                self.yaw_input.handle_event(event)

                if self.target_move_button.handle_event(event):  # "Target J Ref by World" Move button
                    x_value = self.x_input.get_value()
                    y_value = self.y_input.get_value()
                    z_value = self.z_input.get_value()
                    roll_value = self.roll_input.get_value()
                    pitch_value = self.pitch_input.get_value()
                    yaw_value = self.yaw_input.get_value()
                    print(f"Move to Target J: X={x_value}, Y={y_value}, Z={z_value}, Roll={roll_value}, Pitch={pitch_value}, Yaw={yaw_value}")

            # Handle MoveL events
            if self.toggle_buttons[1].active:  # MoveL Mode
                self.position_x_input.handle_event(event)
                self.position_y_input.handle_event(event)
                self.position_z_input.handle_event(event)
                self.position_roll_input.handle_event(event)
                self.position_pitch_input.handle_event(event)
                self.position_yaw_input.handle_event(event)

                self.force_x_input.handle_event(event)
                self.force_y_input.handle_event(event)
                self.force_z_input.handle_event(event)
                self.torque_x_input.handle_event(event)
                self.torque_y_input.handle_event(event)
                self.torque_z_input.handle_event(event)

                if self.movel_move_button.handle_event(event):  # "MoveL" Move button
                    pos_x = self.position_x_input.get_value()
                    pos_y = self.position_y_input.get_value()
                    pos_z = self.position_z_input.get_value()
                    roll = self.position_roll_input.get_value()
                    pitch = self.position_pitch_input.get_value()
                    yaw = self.position_yaw_input.get_value()

                    print(f"Position: X={pos_x}, Y={pos_y}, Z={pos_z}, Roll={roll}, Pitch={pitch}, Yaw={yaw}")

                if self.calculate_button.handle_event(event):
                    force_x = self.force_x_input.get_value()
                    force_y = self.force_y_input.get_value()
                    force_z = self.force_z_input.get_value()
                    torque_x = self.torque_x_input.get_value()
                    torque_y = self.torque_y_input.get_value()
                    torque_z = self.torque_z_input.get_value()

                    print(f"Wrench : [Force: X={force_x}, Y={force_y}, Z={force_z}, Torque: X={torque_x} , Y={torque_y}, Z={torque_z}]")

        # Update cursor visibility for input boxes
        if self.toggle_buttons[0].active:  # MoveJ Mode
            self.x_input.update_cursor()
            self.y_input.update_cursor()
            self.z_input.update_cursor()
            self.roll_input.update_cursor()
            self.pitch_input.update_cursor()
            self.yaw_input.update_cursor()

        if self.toggle_buttons[1].active:  # MoveL Mode
            self.position_x_input.update_cursor()
            self.position_y_input.update_cursor()
            self.position_z_input.update_cursor()
            self.position_roll_input.update_cursor()
            self.position_pitch_input.update_cursor()
            self.position_yaw_input.update_cursor()

            self.force_x_input.update_cursor()
            self.force_y_input.update_cursor()
            self.force_z_input.update_cursor()
            self.torque_x_input.update_cursor()
            self.torque_y_input.update_cursor()
            self.torque_z_input.update_cursor()

        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__=='__main__':
    main()