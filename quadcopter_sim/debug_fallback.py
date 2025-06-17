"""
Centralized debug configuration module with fallback support.
This module provides debug flags for the quadcopter simulation with enhanced
configuration management and runtime update capabilities.
"""

import json
import os
from pathlib import Path

class DebugConfig:
    """Central debug configuration class with dynamic configuration support."""
    
    def __init__(self):
        # Key event debugging
        self.DEBUG_KEY_EVENT = True
        self.DEBUG_IMGUI_CAPTURE = True
        self.DEBUG_ACTION_IGNORE = True
        self.DEBUG_MANUAL_MODE_OFF = True
        
        # Individual key debugging
        self.DEBUG_KEY_W = True
        self.DEBUG_KEY_S = True
        self.DEBUG_KEY_A = True
        self.DEBUG_KEY_D = True
        self.DEBUG_KEY_Q = True
        self.DEBUG_KEY_E = True
        self.DEBUG_KEY_R = True
        self.DEBUG_KEY_F = True
        
        # System debugging
        self.DEBUG_MANUAL_RPMS = False
        self.DEBUG_PHYSICS = False
        self.DEBUG_MANUAL_STATUS = False
        
        # Mark as fallback configuration
        self._is_fallback_config = True
        
        # Try to load configuration from file if available
        self._load_config_if_available()
    
    def _load_config_if_available(self):
        """Load configuration from external file if available."""
        try:
            # Look for config file in various locations
            possible_paths = [
                Path(__file__).parent.parent / 'debug' / 'debug_config.json',
                Path(__file__).parent.parent / 'config' / 'debug.json',
                Path.cwd() / 'debug_config.json'
            ]
            
            for config_path in possible_paths:
                if config_path.exists():
                    with open(config_path, 'r') as f:
                        config_data = json.load(f)
                    
                    # Update configuration with loaded values
                    for key, value in config_data.items():
                        if hasattr(self, key):
                            setattr(self, key, value)
                    break
        except Exception:
            # Silently ignore errors and use defaults
            pass
    
    def save_config(self, config_path=None):
        """Save current configuration to file."""
        if config_path is None:
            config_path = Path(__file__).parent.parent / 'config' / 'debug.json'
        
        config_path.parent.mkdir(parents=True, exist_ok=True)
        
        config_data = {}
        for attr_name in dir(self):
            if attr_name.startswith('DEBUG_') and not callable(getattr(self, attr_name)):
                config_data[attr_name] = getattr(self, attr_name)
        
        with open(config_path, 'w') as f:
            json.dump(config_data, f, indent=2)
    
    def reset_to_defaults(self):
        """Reset all configuration to default values."""
        self.__init__()
    
    def get_all_flags(self):
        """Get all debug flags as a dictionary."""
        return {
            attr: getattr(self, attr)
            for attr in dir(self)
            if attr.startswith('DEBUG_') and not callable(getattr(self, attr))
        }
    
    @classmethod
    def get_instance(cls):
        """Get singleton instance of debug configuration."""
        if not hasattr(cls, '_instance'):
            cls._instance = cls()
        return cls._instance

# Create default instance
_debug_instance = DebugConfig.get_instance()

# Export all debug flags at module level for backward compatibility
DEBUG_KEY_EVENT = _debug_instance.DEBUG_KEY_EVENT
DEBUG_IMGUI_CAPTURE = _debug_instance.DEBUG_IMGUI_CAPTURE
DEBUG_ACTION_IGNORE = _debug_instance.DEBUG_ACTION_IGNORE
DEBUG_MANUAL_MODE_OFF = _debug_instance.DEBUG_MANUAL_MODE_OFF
DEBUG_KEY_W = _debug_instance.DEBUG_KEY_W
DEBUG_KEY_S = _debug_instance.DEBUG_KEY_S
DEBUG_KEY_A = _debug_instance.DEBUG_KEY_A
DEBUG_KEY_D = _debug_instance.DEBUG_KEY_D
DEBUG_KEY_Q = _debug_instance.DEBUG_KEY_Q
DEBUG_KEY_E = _debug_instance.DEBUG_KEY_E
DEBUG_KEY_R = _debug_instance.DEBUG_KEY_R
DEBUG_KEY_F = _debug_instance.DEBUG_KEY_F
DEBUG_MANUAL_RPMS = _debug_instance.DEBUG_MANUAL_RPMS
DEBUG_PHYSICS = _debug_instance.DEBUG_PHYSICS
DEBUG_MANUAL_STATUS = _debug_instance.DEBUG_MANUAL_STATUS
