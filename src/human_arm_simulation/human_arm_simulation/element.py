import pygame

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
        self.txt_surface = SMALL_FONT.render(text, True, BLACK)
        self.active = False
        self.cursor_visible = True  # Toggle cursor visibility
        self.cursor_timer = 0       # Timer for cursor blink
        self.cursor_blink_delay = 2000  # Adjusted delay in milliseconds (slower blink)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)
            self.color = BLUE if self.active else GRAY

        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                self.active = False
                self.color = GRAY
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                # Only allow numbers, minus sign, and decimal point
                if event.unicode in '0123456789.-':
                    # Check for valid decimal point placement
                    if event.unicode == '.':
                        if '.' not in self.text:  # Only allow one decimal point
                            self.text += event.unicode
                    else:
                        # Check decimal places limit
                        if '.' in self.text:
                            decimal_places = len(self.text.split('.')[1]) if len(self.text.split('.')) > 1 else 0
                            if decimal_places < 2:  # Only allow 2 decimal places
                                self.text += event.unicode
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
        
# Add InputBox Class for Text Input
class InputBox_Scroll:
    """Class for input fields (X, Y, Z)."""
    def __init__(self, x, y, width, height, text="", scrollbar=None):
        self.rect = pygame.Rect(x, y, width, height)
        self.color = GRAY
        self.text = text
        self.txt_surface = SMALL_FONT.render(text, True, BLACK)
        self.active = False
        self.cursor_visible = True
        self.cursor_timer = 0
        self.cursor_blink_delay = 2000
        self.scrollbar = scrollbar

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)
            self.color = BLUE if self.active else GRAY

        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                self.active = False
                self.color = GRAY
                self._update_scrollbar()  # Update scrollbar only on Enter
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                # Allow numbers, minus sign, and decimal point
                if event.unicode in '0123456789.-':
                    # Handle minus sign
                    if event.unicode == '-' and len(self.text) == 0:
                        self.text += event.unicode
                    # Handle decimal point
                    elif event.unicode == '.' and '.' not in self.text:
                        self.text += event.unicode
                    # Handle numbers
                    elif event.unicode in '0123456789':
                        if '.' in self.text:
                            decimal_places = len(self.text.split('.')[1]) if len(self.text.split('.')) > 1 else 0
                            if decimal_places < 2:  # Limit to 2 decimal places
                                self.text += event.unicode
                        else:
                            self.text += event.unicode

            # Re-render the text
            self.txt_surface = SMALL_FONT.render(self.text, True, BLACK)

    def _update_scrollbar(self):
        """Update the scrollbar value based on the current input text"""
        if self.scrollbar and self.text:
            try:
                value = float(self.text)
                # Check if value is within limits
                if self.scrollbar.lower_limit <= value <= self.scrollbar.upper_limit:
                    self.scrollbar.set_value(value)
            except ValueError:
                pass  # Ignore invalid number formats

    def get_value(self):
        try:
            return float(self.text)
        except ValueError:
            return 0.0

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.rect, 2)
        screen.blit(self.txt_surface, (self.rect.x + 5, self.rect.y + 5))
        
        if self.active and self.cursor_visible:
            cursor_x = self.rect.x + 5 + self.txt_surface.get_width()
            cursor_y = self.rect.y + 5
            pygame.draw.line(screen, BLACK, (cursor_x, cursor_y), 
                           (cursor_x, cursor_y + self.rect.height - 10), 2)

    def update_cursor(self):
        self.cursor_timer += pygame.time.get_ticks()
        if self.cursor_timer >= self.cursor_blink_delay:
            self.cursor_visible = not self.cursor_visible
            self.cursor_timer = 0


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

        # Add input box for value
        input_box_width = 60
        input_box_height = 30
        self.input_box = InputBox_Scroll(
            x + width//2 - input_box_width//2,  # Center above scrollbar
            y - input_box_height - 5,  # 5 pixels above scrollbar
            input_box_width,
            input_box_height,
            text=f"{self.value:.2f}",
            scrollbar=self
        )

    def draw(self, screen):
        # Draw the track and slider
        pygame.draw.rect(screen, GRAY, self.rect)
        pygame.draw.rect(screen, BLUE, self.slider_rect)

        # Draw the label
        label_surface = SMALL_FONT.render(self.label, True, BLACK)
        label_rect = label_surface.get_rect(midright=(self.rect.left - 70, self.rect.centery))
        screen.blit(label_surface, label_rect)

        # Draw input box instead of static value text
        self.input_box.draw(screen)

        # Draw the value above the scrollbar
        # value_text = f"{self.value:.2f}"  # Format the value to 2 decimal places
        # value_surface = SMALL_FONT.render(value_text, True, BLACK)
        # value_rect = value_surface.get_rect(midbottom=(self.rect.centerx, self.rect.top - 5))
        # screen.blit(value_surface, value_rect)

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
        # Handle input box events
        self.input_box.handle_event(event)
        
        # Update slider if input box value changes
        if self.input_box.active and event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
            new_value = self.input_box.get_value()
            self.set_value(new_value)

        # Handle slider events
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.slider_rect.collidepoint(event.pos):
                self.dragging = True

        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False

        elif event.type == pygame.MOUSEMOTION and self.dragging:
            new_x = min(max(event.pos[0], self.rect.left), self.rect.right - self.slider_rect.width)
            self.slider_rect.x = new_x
            # Update both value and input box text when sliding
            self.value = self.lower_limit + ((self.slider_rect.x - self.rect.left) / 
                (self.rect.width - self.slider_rect.width)) * (self.upper_limit - self.lower_limit)
            self.input_box.text = f"{self.value:.2f}"
            self.input_box.txt_surface = SMALL_FONT.render(self.input_box.text, True, BLACK)

    def set_value(self, new_value):
        """Set the value and update slider position"""
        self.value = min(max(new_value, self.lower_limit), self.upper_limit)
        # Update slider position based on value
        normalized_pos = (self.value - self.lower_limit) / (self.upper_limit - self.lower_limit)
        self.slider_rect.x = self.rect.left + normalized_pos * (self.rect.width - self.slider_rect.width)
        # Update input box
        self.input_box.text = f"{self.value:.2f}"
        self.input_box.txt_surface = SMALL_FONT.render(self.input_box.text, True, BLACK)

    def get_value(self):
        return self.value