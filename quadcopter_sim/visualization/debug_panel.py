"""Debug panel for system diagnostics and debug controls."""

import imgui
from .base_panel import BasePanel


class DebugPanel(BasePanel):
    """Debug control panel with SCADA styling."""
    
    def draw(self, debug_config=None, layout_manager=None):
        """Draw debug control panel with SCADA styling."""
        if debug_config is None:
            return
            
        self.apply_scada_theme()
        
        # Position is handled by layout manager if provided
        if layout_manager is None:
            # Fallback to old positioning
            imgui.set_next_window_position(self.window_width - 430, 400)
        
        imgui.begin("■ SYSTEM DIAGNOSTICS")
          # Show debug info if layout manager is available
        if layout_manager:
            imgui.text_colored(layout_manager.get_panel_debug_info('debug_panel'), 0.5, 0.5, 0.5, 1.0)
            imgui.separator()
        
        self.draw_section_header("DEBUG CONTROLS")
        
        # Key event debugging - more compact layout
        imgui.columns(2, "debug_main")
        
        # Left column - Input and Flight Controls
        imgui.text("INPUT/FLIGHT:")
        changed, debug_config.DEBUG_KEY_EVENT = imgui.checkbox("Key Events", debug_config.DEBUG_KEY_EVENT)
        if changed:
            status = "ENABLED" if debug_config.DEBUG_KEY_EVENT else "DISABLED"
            print(f"[DEBUG] Key Event Debug {status}")
        
        _, debug_config.DEBUG_IMGUI_CAPTURE = imgui.checkbox("ImGui Capture", debug_config.DEBUG_IMGUI_CAPTURE)
        _, debug_config.DEBUG_ACTION_IGNORE = imgui.checkbox("Action Ignore", debug_config.DEBUG_ACTION_IGNORE)
        _, debug_config.DEBUG_MANUAL_MODE_OFF = imgui.checkbox("Manual Mode Off", debug_config.DEBUG_MANUAL_MODE_OFF)
        
        imgui.next_column()
        
        # Right column - System Controls
        imgui.text("SYSTEM:")
        _, debug_config.DEBUG_MANUAL_RPMS = imgui.checkbox("Manual RPMs", debug_config.DEBUG_MANUAL_RPMS)
        _, debug_config.DEBUG_PHYSICS = imgui.checkbox("Physics", debug_config.DEBUG_PHYSICS)
        _, debug_config.DEBUG_MANUAL_STATUS = imgui.checkbox("Manual Status", debug_config.DEBUG_MANUAL_STATUS)
        
        imgui.columns(1)
        
        # Key debug in compact grid
        imgui.spacing()
        imgui.text("KEYS:")
        imgui.columns(8, "key_debug")
        _, debug_config.DEBUG_KEY_W = imgui.checkbox("W", debug_config.DEBUG_KEY_W)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_S = imgui.checkbox("S", debug_config.DEBUG_KEY_S)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_A = imgui.checkbox("A", debug_config.DEBUG_KEY_A)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_D = imgui.checkbox("D", debug_config.DEBUG_KEY_D)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_Q = imgui.checkbox("Q", debug_config.DEBUG_KEY_Q)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_E = imgui.checkbox("E", debug_config.DEBUG_KEY_E)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_R = imgui.checkbox("R", debug_config.DEBUG_KEY_R)
        imgui.next_column()
        _, debug_config.DEBUG_KEY_F = imgui.checkbox("F", debug_config.DEBUG_KEY_F)
        imgui.columns(1)
        
        # Debug status indicator
        imgui.spacing()
        imgui.separator()
        imgui.spacing()
        
        debug_active = any([debug_config.DEBUG_KEY_EVENT, debug_config.DEBUG_IMGUI_CAPTURE,
                           debug_config.DEBUG_ACTION_IGNORE, debug_config.DEBUG_MANUAL_RPMS,
                           debug_config.DEBUG_PHYSICS, debug_config.DEBUG_MANUAL_STATUS])
        
        debug_status = 'warn' if debug_active else 'good'
        self.draw_status_indicator("DEBUG MODE", debug_status)
        
        if debug_active:
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_warn'])
            imgui.text("■ DEBUG LOGGING ACTIVE")
            imgui.pop_style_color()
        else:
            imgui.push_style_color(imgui.COLOR_TEXT, *self.colors['status_good'])
            imgui.text("■ NORMAL OPERATION")
            imgui.pop_style_color()
        
        imgui.end()
