"""Debug panel for system diagnostics and debug controls."""

import imgui
from .base_panel import BasePanel


class DebugPanel(BasePanel):
    """Debug control panel with SCADA styling."""
    
    def draw(self, debug_config=None):
        """Draw debug control panel with SCADA styling."""
        if debug_config is None:
            return
            
        self.apply_scada_theme()
        
        # Position at bottom right
        imgui.set_next_window_position(self.window_width - 450, self.window_height - 300)
        imgui.set_next_window_size(440, 290)
        
        imgui.begin("■ SYSTEM DIAGNOSTICS", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        
        self.draw_section_header("DEBUG CONTROLS")
        
        # Key event debugging
        imgui.text("INPUT SYSTEM:")
        changed, debug_config.DEBUG_KEY_EVENT = imgui.checkbox("KEY EVENT DEBUG", debug_config.DEBUG_KEY_EVENT)
        if changed:
            status = "ENABLED" if debug_config.DEBUG_KEY_EVENT else "DISABLED"
            print(f"[DEBUG] Key Event Debug {status}")
        
        changed, debug_config.DEBUG_IMGUI_CAPTURE = imgui.checkbox("IMGUI CAPTURE DEBUG", debug_config.DEBUG_IMGUI_CAPTURE)
        changed, debug_config.DEBUG_ACTION_IGNORE = imgui.checkbox("ACTION IGNORE DEBUG", debug_config.DEBUG_ACTION_IGNORE)
        
        imgui.spacing()
        imgui.text("FLIGHT CONTROLS:")
        
        # Manual mode debugging
        changed, debug_config.DEBUG_MANUAL_MODE_OFF = imgui.checkbox("MANUAL MODE OFF DEBUG", debug_config.DEBUG_MANUAL_MODE_OFF)
        
        # Individual key debugging - arranged in a grid
        imgui.text("KEY DEBUG:")
        imgui.columns(4, "key_debug")
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
        
        imgui.spacing()
        imgui.text("SYSTEM DEBUG:")
        _, debug_config.DEBUG_MANUAL_RPMS = imgui.checkbox("MANUAL RPMS DEBUG", debug_config.DEBUG_MANUAL_RPMS)
        _, debug_config.DEBUG_PHYSICS = imgui.checkbox("PHYSICS DEBUG", debug_config.DEBUG_PHYSICS)
        _, debug_config.DEBUG_MANUAL_STATUS = imgui.checkbox("MANUAL STATUS DEBUG", debug_config.DEBUG_MANUAL_STATUS)
        
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
