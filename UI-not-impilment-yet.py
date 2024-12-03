import pygame

# Initialize PyGame
pygame.init()

# Screen Dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 1000

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
BLUE = (0, 122, 255)
ORANGE = (255, 165, 0)

# Fonts
pygame.font.init()
FONT = pygame.font.SysFont('Arial', 30)
SMALL_FONT = pygame.font.SysFont('Arial', 20)
MED_FONT = pygame.font.SysFont('Arial',30)


class ToggleButton:
    """Toggle Button Class for buttons like MoveJ, MoveL, etc."""
    def __init__(self, x, y, width, height, text, active=False, color=BLUE):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.active = active
        self.color = color

    def draw(self, screen):
        color = self.color if self.active else GRAY
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)  # Border
        text_surface = FONT.render(self.text, True, WHITE if self.active else BLACK)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                return True  # Button was clicked
        return False


class Button:
    """Normal Button Class for the Move button."""
    def __init__(self, x, y, width, height, text, color=ORANGE):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)  # Border
        text_surface = FONT.render(self.text, True, WHITE)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                return True  # Button was clicked
        return False
# Add InputBox Class for Text Input
class InputBox:
    """Class for input fields (X, Y, Z)."""
    def __init__(self, x, y, width, height, text=""):
        self.rect = pygame.Rect(x, y, width, height)
        self.color = GRAY
        self.text = text
        self.txt_surface = FONT.render(text, True, BLACK)
        self.active = False
        self.cursor_visible = True  # Toggle cursor visibility
        self.cursor_timer = 0       # Timer for cursor blink
        self.cursor_blink_delay = 2000  # Adjusted delay in milliseconds (slower blink)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            # Toggle active state if the user clicks inside the input box
            self.active = self.rect.collidepoint(event.pos)
            self.color = BLUE if self.active else GRAY

        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                self.active = False
                self.color = GRAY
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                self.text += event.unicode
            # Re-render the text
            self.txt_surface = FONT.render(self.text, True, BLACK)

    def draw(self, screen):
        # Draw the box
        pygame.draw.rect(screen, self.color, self.rect, 2)
        # Draw the text inside the box
        screen.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))

        # Draw the cursor
        if self.active and self.cursor_visible:
            cursor_x = self.rect.x + 5 + self.txt_surface.get_width()
            cursor_y = self.rect.y + 5
            pygame.draw.line(screen, BLACK, (cursor_x, cursor_y), (cursor_x, cursor_y + self.rect.height - 10), 2)

    def update_cursor(self):
        """Blink the cursor at a slower rate (based on cursor_blink_delay)."""
        self.cursor_timer += pygame.time.get_ticks()
        if self.cursor_timer >= self.cursor_blink_delay:  # Slower blink (800ms)
            self.cursor_visible = not self.cursor_visible
            self.cursor_timer = 0

    def get_value(self):
        # Return the value of the input as float (default to 0 if empty)
        try:
            return float(self.text)
        except ValueError:
            return 0.0
class ScrollBar:
    """Scrollbar Class to handle joint sliders."""
    def __init__(self, x, y, width, height, label, lower_limit, upper_limit):
        self.rect = pygame.Rect(x, y, width, height)  # Outer rectangle (track)
        self.slider_rect = pygame.Rect(x, y, height, height)  # Slider button
        self.value = lower_limit  # Default value starts at lower limit
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.dragging = False
        self.label = label

    def draw(self, screen):
        # Draw the track
        pygame.draw.rect(screen, GRAY, self.rect)
        # Draw the slider button
        pygame.draw.rect(screen, BLUE, self.slider_rect)

        # Draw the label
        label_surface = SMALL_FONT.render(self.label, True, BLACK)
        label_rect = label_surface.get_rect(midright=(self.rect.left - 70, self.rect.centery))
        screen.blit(label_surface, label_rect)

        # Draw the value above the scrollbar
        value_text = f"{self.value:.2f}"  # Format the value to 2 decimal places
        value_surface = SMALL_FONT.render(value_text, True, BLACK)
        value_rect = value_surface.get_rect(midbottom=(self.rect.centerx, self.rect.top - 5))
        screen.blit(value_surface, value_rect)

        # Adjust gap for lower limit if negative
        lower_limit_gap_x = 10 if self.lower_limit < 0 else 0
        lower_limit_text = f"{self.lower_limit:.2f}"
        lower_limit_surface = SMALL_FONT.render(lower_limit_text, True, BLACK)
        lower_limit_rect = lower_limit_surface.get_rect(midleft=(self.rect.left - 40 - lower_limit_gap_x, self.rect.centery))
        screen.blit(lower_limit_surface, lower_limit_rect)

        # Draw the upper limit further to the right of the scrollbar
        upper_limit_text = f"{self.upper_limit:.2f}"
        upper_limit_surface = SMALL_FONT.render(upper_limit_text, True, BLACK)
        upper_limit_rect = upper_limit_surface.get_rect(midright=(self.rect.right + 40, self.rect.centery))
        screen.blit(upper_limit_surface, upper_limit_rect)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.slider_rect.collidepoint(event.pos):
                self.dragging = True  # Start dragging the slider

        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False  # Stop dragging

        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                # Update slider position within the track bounds
                new_x = min(max(event.pos[0], self.rect.left), self.rect.right - self.slider_rect.width)
                self.slider_rect.x = new_x
                # Map the slider position to the value range (lower_limit to upper_limit)
                self.value = self.lower_limit + ((self.slider_rect.x - self.rect.left) / (self.rect.width - self.slider_rect.width)) * (self.upper_limit - self.lower_limit)

    def get_value(self):
        return self.value


def draw_joint_values(screen, move_button, scrollbars):
    """Display joint values below the Move button."""
    values = [f"{scrollbar.label}: {scrollbar.get_value():.2f}" for scrollbar in scrollbars]
    for i, value in enumerate(values):
        value_surface = SMALL_FONT.render(value, True, BLACK)
        value_rect = value_surface.get_rect(topleft=(move_button.rect.left, move_button.rect.bottom + 10 + i * 25))
        screen.blit(value_surface, value_rect)


# Initialize the screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Human Arm Robot UI")

# Define UI elements
header_height = 100
button_width = 200
button_height = 60
button_spacing = 40
button_y = header_height + 20
toggle_buttons = [
    ToggleButton(SCREEN_WIDTH // 2 - button_width - button_spacing // 2, button_y, button_width, button_height, "MoveJ", active=True),
    ToggleButton(SCREEN_WIDTH // 2 + button_spacing // 2, button_y, button_width, button_height, "MoveL")
]

section_y = header_height + 150
# Define static titles instead of toggle buttons
movej_titles = [
    {"text": "Move by Q", "pos": (SCREEN_WIDTH // 4, section_y + button_height // 2)},
    {"text": "Target J Ref by World", "pos": (3 * SCREEN_WIDTH // 4, section_y + button_height // 2)}
]

left_section_top = section_y + button_height + 50
scrollbar_width = 100
scrollbar_height = 20
scrollbar_x = 150
scrollbar_spacing = 50

joint_limits = [
    (-0.87, 3.14),
    (0, 1.57),
    (0, 3.14),
    (0, 2.16),
    (-1.57, 1.57),
    (-1.57, 1.04),
    (-0.34, 0.69)
]

scrollbars = []
y_pos = left_section_top
for i, (lower, upper) in enumerate(joint_limits):
    scrollbars.append(ScrollBar(scrollbar_x, y_pos, scrollbar_width, scrollbar_height, f"q{i+1}", lower, upper))
    y_pos += scrollbar_spacing

move_button = Button(scrollbar_x, y_pos + 20, 100, 50, "Move")

# Extend Main Program
x_input = InputBox(SCREEN_WIDTH // 2 + 100, 300+50, 200, 40)  # Input for X
y_input = InputBox(SCREEN_WIDTH // 2 + 100, 360+50, 200, 40)  # Input for Y
z_input = InputBox(SCREEN_WIDTH // 2 + 100, 420+50, 200, 40)  # Input for Z
target_move_button = Button(SCREEN_WIDTH // 2 +150, 700+50, 100, 50, "Move")  # Move button for Target J Ref by World


roll_input = InputBox(SCREEN_WIDTH // 2 + 100, 500+50, 200, 40)  # Input for Roll
pitch_input = InputBox(SCREEN_WIDTH // 2 + 100, 560+50, 200, 40)  # Input for Pitch
yaw_input = InputBox(SCREEN_WIDTH // 2 + 100, 620+50, 200, 40)  # Input for Yaw

# InputBox instances for Position
position_x_input = InputBox(SCREEN_WIDTH // 6 - 50, 250+100, 100, 40, "")  # Input for Position X
position_y_input = InputBox(SCREEN_WIDTH // 6 - 50, 310+100, 100, 40, "")  # Input for Position Y
position_z_input = InputBox(SCREEN_WIDTH // 6 - 50, 370+100, 100, 40, "")  # Input for Position Z

position_roll_input = InputBox(SCREEN_WIDTH // 6 - 50, 430+120, 100, 40, "")
position_pitch_input = InputBox(SCREEN_WIDTH // 6 - 50, 490+120, 100, 40, "")
position_yaw_input = InputBox(SCREEN_WIDTH // 6 - 50, 550+120, 100, 40, "")

movel_move_button = Button(SCREEN_WIDTH // 6 - 50, 650 + 120, 100, 50, "Move")  # Positioned in the Joint Effort section


# InputBox instances for Force
force_x_input = InputBox(SCREEN_WIDTH // 2 -20, 250+100, 100, 40, "")  # Input for Force X
force_y_input = InputBox(SCREEN_WIDTH // 2  -20, 310+100, 100, 40, "")  # Input for Force Y
force_z_input = InputBox(SCREEN_WIDTH // 2  -20, 370+100, 100, 40, "")  # Input for Force Z

torque_x_input = InputBox(SCREEN_WIDTH // 2  -20, 450+100, 100, 40, "")  # Input for Force X
torque_y_input = InputBox(SCREEN_WIDTH // 2  -20, 510+100, 100, 40, "")  # Input for Force Y
torque_z_input = InputBox(SCREEN_WIDTH // 2  -20, 570+100, 100, 40, "")  # Input for Force Z

calculate_button = Button(SCREEN_WIDTH  // 2 + 50, 650 + 120, 150, 50, "Calculate")  # Positioned in the Joint Effort section


# Placeholder text values for Joint Effort
joint_effort_values = ["Joint 1: 0.00", "Joint 2: 0.00", "Joint 3: 0.00", "Joint 4: 0.00", "Joint 5: 0.00", "Joint 6: 0.00"]


# Add a Move Button for MoveL in the 3rd section

# Main Loop
show_values = False
running = True
# Main Loop (updated portion)
while running:
    screen.fill(WHITE)

    # Draw Header
    title_surface = FONT.render("Human Arm Robot", True, BLACK)
    title_rect = title_surface.get_rect(center=(SCREEN_WIDTH // 2, header_height // 2))
    screen.blit(title_surface, title_rect)

    # Draw Top-Level Toggle Buttons
    for button in toggle_buttons:
        button.draw(screen)

    if toggle_buttons[0].active:  # MoveJ Mode
        # Draw vertical line to split into 2 sections
        pygame.draw.line(screen, BLACK, (SCREEN_WIDTH // 2, header_height + 100), (SCREEN_WIDTH // 2, SCREEN_HEIGHT), 3)

        # Draw MoveJ Titles
        for title in movej_titles:
            title_surface = FONT.render(title["text"], True, BLACK)
            title_rect = title_surface.get_rect(center=title["pos"])
            screen.blit(title_surface, title_rect)

        # ---- Move by Q Section ----
        # Draw scrollbars
        for scrollbar in scrollbars:
            scrollbar.draw(screen)

        # Draw the Move button for Move by Q
        move_button.draw(screen)

        # If the Move button was clicked, display the values
        if show_values:
            # Retrieve and display joint values from scrollbars
            values = [f"{scrollbar.label}: {scrollbar.get_value():.2f}" for scrollbar in scrollbars]
            for i, value in enumerate(values):
                value_surface = SMALL_FONT.render(value, True, BLACK)
                value_rect = value_surface.get_rect(topleft=(move_button.rect.left, move_button.rect.bottom + 10 + i * 25))
                screen.blit(value_surface, value_rect)

        # ---- Target J Ref by World Section ----
        # Draw input fields for X, Y, Z
        x_input.draw(screen)
        y_input.draw(screen)
        z_input.draw(screen)

        # Draw input fields for Roll, Pitch, Yaw
        roll_input.draw(screen)
        pitch_input.draw(screen)
        yaw_input.draw(screen)

        # Draw the Move button for Target J Ref by World
        target_move_button.draw(screen)

        # Draw labels for X, Y, Z
        x_label = SMALL_FONT.render("X:", True, BLACK)
        screen.blit(x_label, (x_input.rect.x - 30, x_input.rect.y + 10))

        y_label = SMALL_FONT.render("Y:", True, BLACK)
        screen.blit(y_label, (y_input.rect.x - 30, y_input.rect.y + 10))

        z_label = SMALL_FONT.render("Z:", True, BLACK)
        screen.blit(z_label, (z_input.rect.x - 30, z_input.rect.y + 10))

        # Draw labels for Roll, Pitch, Yaw
        roll_label = SMALL_FONT.render("Roll:", True, BLACK)
        screen.blit(roll_label, (roll_input.rect.x - 60, roll_input.rect.y + 10))

        pitch_label = SMALL_FONT.render("Pitch:", True, BLACK)
        screen.blit(pitch_label, (pitch_input.rect.x - 60, pitch_input.rect.y + 10))

        yaw_label = SMALL_FONT.render("Yaw:", True, BLACK)
        screen.blit(yaw_label, (yaw_input.rect.x - 60, yaw_input.rect.y + 10))

    elif toggle_buttons[1].active:  # MoveL Mode
        # Draw dividing lines for 3 sections
        pygame.draw.line(screen, BLACK, (SCREEN_WIDTH // 3, header_height + 100), (SCREEN_WIDTH // 3, SCREEN_HEIGHT), 3)
        # pygame.draw.line(screen, BLACK, (2 * SCREEN_WIDTH // 3, header_height + 100), (2 * SCREEN_WIDTH // 3, SCREEN_HEIGHT), 3)

        # Draw titles for each section
        position_title = FONT.render("Position", True, BLACK)
        position_title_rect = position_title.get_rect(center=(SCREEN_WIDTH // 6, header_height + 150))
        screen.blit(position_title, position_title_rect)

        Wrench_title = FONT.render("Wrench", True, BLACK)
        Wrench_title_rect = Wrench_title.get_rect(center=(SCREEN_WIDTH // 2, header_height + 150))
        screen.blit(Wrench_title, Wrench_title_rect)

        joint_effort_title = FONT.render("Joint Effort", True, BLACK)
        joint_effort_title_rect = joint_effort_title.get_rect(center=(5 * SCREEN_WIDTH // 6, header_height + 150))
        screen.blit(joint_effort_title, joint_effort_title_rect)

        # ---- 1st Section: Position ----
        # Draw input fields for Position X, Y, Z
        position_x_input.draw(screen)
        position_y_input.draw(screen)
        position_z_input.draw(screen)

        position_roll_input.draw(screen)
        position_pitch_input.draw(screen)
        position_yaw_input.draw(screen)

        pos_x_label = SMALL_FONT.render("X:", True, BLACK)
        screen.blit(pos_x_label, (position_x_input.rect.x - 30, position_x_input.rect.y + 10))

        pos_y_label = SMALL_FONT.render("Y:", True, BLACK)
        screen.blit(pos_y_label, (position_y_input.rect.x - 30, position_y_input.rect.y + 10))

        pos_z_label = SMALL_FONT.render("Z:", True, BLACK)
        screen.blit(pos_z_label, (position_z_input.rect.x - 30, position_z_input.rect.y + 10))

        pos_roll_label = SMALL_FONT.render("Roll:", True, BLACK)
        screen.blit(pos_roll_label, (position_roll_input.rect.x - 60, position_roll_input.rect.y + 10))

        pos_pitch_label = SMALL_FONT.render("Pitch:", True, BLACK)
        screen.blit(pos_pitch_label, (position_pitch_input.rect.x - 60, position_pitch_input.rect.y + 10))

        pos_yaw_label = SMALL_FONT.render("Yaw:", True, BLACK)
        screen.blit(pos_yaw_label, (position_yaw_input.rect.x - 60, position_yaw_input.rect.y + 10))

        # ---- 2nd Section: Force ----
        # Draw input fields for Force X, Y, Z
        force_x_input.draw(screen)
        force_y_input.draw(screen)
        force_z_input.draw(screen)
        torque_x_input.draw(screen)
        torque_y_input.draw(screen)
        torque_z_input.draw(screen)

        force_x_label = SMALL_FONT.render("Force X:", True, BLACK)
        screen.blit(force_x_label, (force_x_input.rect.x - 90, force_x_input.rect.y + 10))

        force_y_label = SMALL_FONT.render("Force Y:", True, BLACK)
        screen.blit(force_y_label, (force_y_input.rect.x - 90, force_y_input.rect.y + 10))

        force_z_label = SMALL_FONT.render("Force Z:", True, BLACK)
        screen.blit(force_z_label, (force_z_input.rect.x - 90, force_z_input.rect.y + 10))

        torque_x_label = SMALL_FONT.render("Torque X:", True, BLACK)
        screen.blit(torque_x_label, (torque_x_input.rect.x - 90, torque_x_input.rect.y + 10))

        torque_y_label = SMALL_FONT.render("Torque Y:", True, BLACK)
        screen.blit(torque_y_label, (torque_y_input.rect.x - 90, torque_y_input.rect.y + 10))

        torque_z_label = SMALL_FONT.render("Torque Z:", True, BLACK)
        screen.blit(torque_z_label, (torque_z_input.rect.x - 90, torque_z_input.rect.y + 10))

        # ---- 3rd Section: Joint Effort ----
        # Display text values for Joint Efforts
        for i, effort in enumerate(joint_effort_values):
            effort_surface = MED_FONT.render(effort, True, BLACK)
            effort_rect = effort_surface.get_rect(topleft=(5 * SCREEN_WIDTH // 6 - 90 , 250 + 100 + i * 50))
            screen.blit(effort_surface, effort_rect)

        # Draw the Move button for MoveL
        movel_move_button.draw(screen)
        calculate_button.draw(screen)

    # Event Handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Handle toggle buttons
        for button in toggle_buttons:
            if button.handle_event(event):
                for other_button in toggle_buttons:
                    other_button.active = False
                button.active = True

        # Handle MoveJ events
        if toggle_buttons[0].active:  # MoveJ Mode
            # Handle scrollbars in "Move by Q"
            for scrollbar in scrollbars:
                scrollbar.handle_event(event)

            if move_button.handle_event(event):  # "Move by Q" Move button
                show_values = True
                values = [f"{scrollbar.label}: {scrollbar.get_value():.2f}" for scrollbar in scrollbars]
                print("Move by Q Joint Values:")
                for value in values:
                    print(value)

            # Handle inputs in "Target J Ref by World"
            x_input.handle_event(event)
            y_input.handle_event(event)
            z_input.handle_event(event)
            roll_input.handle_event(event)
            pitch_input.handle_event(event)
            yaw_input.handle_event(event)

            if target_move_button.handle_event(event):  # "Target J Ref by World" Move button
                x_value = x_input.get_value()
                y_value = y_input.get_value()
                z_value = z_input.get_value()
                roll_value = roll_input.get_value()
                pitch_value = pitch_input.get_value()
                yaw_value = yaw_input.get_value()
                print(f"Move to Target J: X={x_value}, Y={y_value}, Z={z_value}, Roll={roll_value}, Pitch={pitch_value}, Yaw={yaw_value}")

        # Handle MoveL events
        if toggle_buttons[1].active:  # MoveL Mode
            position_x_input.handle_event(event)
            position_y_input.handle_event(event)
            position_z_input.handle_event(event)
            position_roll_input.handle_event(event)
            position_pitch_input.handle_event(event)
            position_yaw_input.handle_event(event)

            force_x_input.handle_event(event)
            force_y_input.handle_event(event)
            force_z_input.handle_event(event)
            torque_x_input.handle_event(event)
            torque_y_input.handle_event(event)
            torque_z_input.handle_event(event)

            if movel_move_button.handle_event(event):  # "MoveL" Move button
                pos_x = position_x_input.get_value()
                pos_y = position_y_input.get_value()
                pos_z = position_z_input.get_value()
                roll = position_roll_input.get_value()
                pitch = position_pitch_input.get_value()
                yaw = position_yaw_input.get_value()

                # force_x = force_x_input.get_value()
                # force_y = force_y_input.get_value()
                # force_z = force_z_input.get_value()

                print(f"Position: X={pos_x}, Y={pos_y}, Z={pos_z}, Roll={roll}, Pitch={pitch}, Yaw={yaw}")
                # print(f"Force: X={force_x}, Y={force_y}, Z={force_z}")
            if calculate_button.handle_event(event):
                force_x = force_x_input.get_value()
                force_y = force_y_input.get_value()
                force_z = force_z_input.get_value()
                torque_x = torque_x_input.get_value()
                torque_y = torque_y_input.get_value()
                torque_z = torque_z_input.get_value()

                print(f"Wrench : [Force: X={force_x}, Y={force_y}, Z={force_z}, Torque: X={torque_x} , Y={torque_y}, Z={torque_z}]")

    # Update cursor visibility for input boxes
    if toggle_buttons[0].active:  # MoveJ Mode
        x_input.update_cursor()
        y_input.update_cursor()
        z_input.update_cursor()
        roll_input.update_cursor()
        pitch_input.update_cursor()
        yaw_input.update_cursor()

    if toggle_buttons[1].active:  # MoveL Mode
        position_x_input.update_cursor()
        position_y_input.update_cursor()
        position_z_input.update_cursor()
        position_roll_input.update_cursor()
        position_pitch_input.update_cursor()
        position_yaw_input.update_cursor()

        force_x_input.update_cursor()
        force_y_input.update_cursor()
        force_z_input.update_cursor()
        torque_x_input.update_cursor()
        torque_y_input.update_cursor()
        torque_z_input.update_cursor()

    pygame.display.flip()

pygame.quit()