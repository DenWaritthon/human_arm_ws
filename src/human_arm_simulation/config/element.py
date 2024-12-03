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