"""Base panel class with SCADA styling and common utilities."""

import imgui


class BasePanel:
    """Base class for all SCADA-styled UI panels."""
    
    def __init__(self, window_width=1200, window_height=800):
        self.window_width = window_width
        self.window_height = window_height
        
        # SCADA Color Scheme
        self.colors = {
            'bg_dark': (0.12, 0.12, 0.15, 1.0),       # Dark background
            'bg_panel': (0.18, 0.18, 0.22, 0.95),     # Panel background
            'bg_header': (0.25, 0.25, 0.30, 1.0),     # Header background
            'text_primary': (0.95, 0.95, 0.95, 1.0),  # Primary text
            'text_secondary': (0.8, 0.8, 0.8, 1.0),   # Secondary text
            'accent_blue': (0.2, 0.6, 1.0, 1.0),      # System blue
            'accent_cyan': (0.0, 0.8, 0.9, 1.0),      # Cyan accents
            'status_good': (0.2, 0.8, 0.2, 1.0),      # Green for good status
            'status_warn': (1.0, 0.8, 0.0, 1.0),      # Yellow for warnings
            'status_alarm': (1.0, 0.3, 0.3, 1.0),     # Red for alarms
            'button_normal': (0.3, 0.3, 0.4, 1.0),    # Normal button
            'button_hover': (0.4, 0.4, 0.5, 1.0),     # Hover button
            'button_active': (0.2, 0.6, 1.0, 1.0),    # Active button
            'button_emergency': (0.8, 0.2, 0.2, 1.0), # Emergency button
            'gauge_bg': (0.15, 0.15, 0.2, 1.0),       # Gauge background
            'gauge_fill': (0.2, 0.6, 1.0, 1.0),       # Gauge fill
        }        # Button dimensions for consistency - auto-sizing friendly
        self.button_size = {
            'small': (100, 30),
            'medium': (140, 35),
            'large': (180, 40),
            'emergency': (160, 45),
        }
    
    def apply_scada_theme(self):
        """Apply SCADA industrial theme to ImGui."""
        style = imgui.get_style()
        
        # Window styling
        style.window_rounding = 2.0
        style.window_border_size = 1.0
        style.window_padding = (12, 8)
        style.frame_rounding = 3.0
        style.frame_border_size = 1.0
        style.frame_padding = (8, 4)
        style.item_spacing = (8, 6)
        style.item_inner_spacing = (6, 4)
        style.indent_spacing = 20.0
        style.scrollbar_size = 16.0
        style.scrollbar_rounding = 3.0
        style.grab_min_size = 12.0
        style.grab_rounding = 3.0
        style.button_text_align = (0.5, 0.5)
        
        # Colors
        colors = style.colors
        colors[imgui.COLOR_WINDOW_BACKGROUND] = self.colors['bg_panel']
        colors[imgui.COLOR_CHILD_BACKGROUND] = self.colors['bg_dark']
        colors[imgui.COLOR_POPUP_BACKGROUND] = self.colors['bg_panel']
        colors[imgui.COLOR_FRAME_BACKGROUND] = self.colors['gauge_bg']
        colors[imgui.COLOR_FRAME_BACKGROUND_HOVERED] = self.colors['button_hover']
        colors[imgui.COLOR_FRAME_BACKGROUND_ACTIVE] = self.colors['button_active']
        colors[imgui.COLOR_TITLE_BACKGROUND] = self.colors['bg_header']
        colors[imgui.COLOR_TITLE_BACKGROUND_ACTIVE] = self.colors['accent_blue']
        colors[imgui.COLOR_TITLE_BACKGROUND_COLLAPSED] = self.colors['bg_header']
        colors[imgui.COLOR_BUTTON] = self.colors['button_normal']
        colors[imgui.COLOR_BUTTON_HOVERED] = self.colors['button_hover']
        colors[imgui.COLOR_BUTTON_ACTIVE] = self.colors['button_active']
        colors[imgui.COLOR_HEADER] = self.colors['accent_blue']
        colors[imgui.COLOR_HEADER_HOVERED] = (0.3, 0.7, 1.0, 0.8)
        colors[imgui.COLOR_HEADER_ACTIVE] = (0.2, 0.6, 1.0, 1.0)
        colors[imgui.COLOR_TEXT] = self.colors['text_primary']
        colors[imgui.COLOR_TEXT_DISABLED] = self.colors['text_secondary']
        colors[imgui.COLOR_BORDER] = self.colors['accent_cyan']
        colors[imgui.COLOR_BORDER_SHADOW] = (0.0, 0.0, 0.0, 0.5)
        colors[imgui.COLOR_PLOT_LINES] = self.colors['accent_cyan']
        colors[imgui.COLOR_PLOT_HISTOGRAM] = self.colors['accent_blue']
        colors[imgui.COLOR_SLIDER_GRAB] = self.colors['accent_blue']
        colors[imgui.COLOR_SLIDER_GRAB_ACTIVE] = self.colors['accent_cyan']
        colors[imgui.COLOR_CHECK_MARK] = self.colors['status_good']
    
    def draw_status_indicator(self, label, status, size=(20, 20)):
        """Draw a circular status indicator light."""
        if status == 'good':
            color = self.colors['status_good']
        elif status == 'warn':
            color = self.colors['status_warn']
        elif status == 'alarm':
            color = self.colors['status_alarm']
        else:
            color = self.colors['text_secondary']
        
        # Draw colored circle
        draw_list = imgui.get_window_draw_list()
        pos = imgui.get_cursor_screen_pos()
        center = (pos[0] + size[0]/2, pos[1] + size[1]/2)
        radius = min(size[0], size[1]) / 2 - 2
        
        # Convert color to ImU32
        color_u32 = imgui.get_color_u32_rgba(*color)
        draw_list.add_circle_filled(center[0], center[1], radius, color_u32)
        draw_list.add_circle(center[0], center[1], radius, imgui.get_color_u32_rgba(1,1,1,0.3), 0, 1.0)
        
        imgui.dummy(size[0], size[1])
        imgui.same_line()
        imgui.text(label)
    
    def draw_large_button(self, label, size=None, color_scheme='normal'):
        """Draw a large SCADA-style button."""
        if size is None:
            size = self.button_size['medium']
            
        if color_scheme == 'emergency':
            imgui.push_style_color(imgui.COLOR_BUTTON, *self.colors['button_emergency'])
            imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 1.0, 0.4, 0.4, 1.0)
            imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.9, 0.1, 0.1, 1.0)
        elif color_scheme == 'active':
            imgui.push_style_color(imgui.COLOR_BUTTON, *self.colors['button_active'])
            imgui.push_style_color(imgui.COLOR_BUTTON_HOVERED, 0.3, 0.7, 1.0, 1.0)
            imgui.push_style_color(imgui.COLOR_BUTTON_ACTIVE, 0.1, 0.5, 0.9, 1.0)
        
        result = imgui.button(label, *size)
        
        if color_scheme in ['emergency', 'active']:
            imgui.pop_style_color(3)
            
        return result
    
    def draw_section_header(self, title):
        """Draw a section header with SCADA styling."""
        imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['accent_cyan'])
        imgui.text(f"■ {title.upper()}")
        imgui.pop_style_color()
        imgui.separator()
        imgui.spacing()
    
    def draw_value_display(self, label, value, unit="", status='normal', width=120):
        """Draw a value display with label and status coloring."""
        if status == 'good':
            color = self.colors['status_good']
        elif status == 'warn':
            color = self.colors['status_warn']
        elif status == 'alarm':
            color = self.colors['status_alarm']
        else:
            color = self.colors['text_primary']
        
        imgui.text(f"{label}:")
        imgui.same_line(width)
        imgui.push_style_color(imgui.COLOR_TEXT, *color)
        imgui.text(f"{value} {unit}")
        imgui.pop_style_color()
    
    def update_window_size(self, width, height):
        """Update the window dimensions."""
        self.window_width = width
        self.window_height = height
