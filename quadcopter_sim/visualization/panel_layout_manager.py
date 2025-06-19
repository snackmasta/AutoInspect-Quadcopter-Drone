"""Manages the layout and positioning of UI panels to avoid overlap."""

import imgui

class PanelLayoutManager:
    """Manages the automatic positioning of UI panels to prevent overlap."""
    
    def __init__(self, window_width, window_height, padding=10):
        self.window_width = window_width
        self.window_height = window_height
        self.padding = padding
        self.panels = {}
        self.render_order = []
        
    def register_panel(self, name, preferred_side='left', min_size=(300, 200)):
        """Register a panel with the layout manager."""
        self.panels[name] = {
            'preferred_side': preferred_side,
            'min_size': min_size,
            'position': (0, 0),
            'size': min_size,
            'rendered': False,
            'user_positioned': False
        }
        if name not in self.render_order:
            self.render_order.append(name)
    
    def begin_panel(self, name):
        """Start rendering a panel at its calculated position."""
        if name not in self.panels:
            self.register_panel(name)
        
        panel = self.panels[name]
        x, y = panel['position']
        
        # Only set position if panel is not being dragged by user
        # Check if this is the first frame for this panel or if we should force position
        if not panel.get('user_positioned', False):
            imgui.set_next_window_position(x, y)
        
        # Set size constraints to allow content-based sizing
        imgui.set_next_window_size_constraints((100, 50), (800, 1000))
        # Mark that we're about to render this panel
        panel['rendered'] = True
        
    def end_panel(self, name):
        """Finish rendering a panel and update its size after imgui.end()."""
        if name in self.panels:            # Store the name to measure size after imgui.end()
            self.panels[name]['needs_size_update'] = True

    def capture_panel_size(self, name):
        """Capture the final size of a panel. Call this immediately after imgui.end()."""
        if name in self.panels:
            size = imgui.get_window_size()
            pos = imgui.get_window_position()
            
            # Check if user has moved the panel
            expected_pos = self.panels[name]['position']
            if abs(pos.x - expected_pos[0]) > 5 or abs(pos.y - expected_pos[1]) > 5:
                # User has moved the panel, mark it as user-positioned
                self.panels[name]['user_positioned'] = True
                self.panels[name]['position'] = (pos.x, pos.y)
            
            self.panels[name]['size'] = (size.x, size.y)
            self.panels[name]['needs_size_update'] = False

    def update_layout(self):
        """Stack panels using the sizes measured in the previous frame."""
        left_panels = []
        right_panels = []
        for name in self.render_order:
            if name in self.panels:
                side = self.panels[name]['preferred_side']
                if side == 'right':
                    right_panels.append(name)
                else:
                    left_panels.append(name)
        # Stack left panels
        current_y = self.padding
        for name in left_panels:
            if name in self.panels:
                panel = self.panels[name]
                panel['position'] = (self.padding, current_y)
                # Use measured size if available, else min_size
                height = panel['size'][1] if 'size' in panel else panel['min_size'][1]
                current_y += height + self.padding
        # Stack right panels
        current_y = self.padding
        for name in right_panels:
            if name in self.panels:
                panel = self.panels[name]
                width = panel['size'][0] if 'size' in panel else panel['min_size'][0]
                height = panel['size'][1] if 'size' in panel else panel['min_size'][1]
                x = self.window_width - width - self.padding
                panel['position'] = (x, current_y)
                current_y += height + self.padding

    def update_window_size(self, width, height):
        """Update window dimensions and recalculate layout."""
        self.window_width = width
        self.window_height = height
        self.update_layout()

    def get_panel_debug_info(self, name):
        """Get debug information for a panel."""
        if name in self.panels:
            panel = self.panels[name]
            size = panel.get('size', (0, 0))
            stored_pos = panel.get('position', (0, 0))
            user_pos = panel.get('user_positioned', False)
            
            # Get current real-time position from ImGui
            try:
                current_pos = imgui.get_window_position()
                pos_str = f"{current_pos.x:.0f},{current_pos.y:.0f}"
            except:
                # Fallback to stored position if ImGui call fails
                pos_str = f"{stored_pos[0]:.0f},{stored_pos[1]:.0f}"
            
            return f"Size: {size[0]:.0f}x{size[1]:.0f} | Pos: {pos_str} | User: {user_pos}"
        return "Panel not found"
