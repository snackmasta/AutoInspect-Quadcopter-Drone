"""
Universal debug import utility.
Provides a robust way to import debug configuration with automatic fallback.
Enhanced version with better error handling and configuration management.
"""

import sys
import os
import logging
from pathlib import Path

# Setup logger for debug import issues
logger = logging.getLogger(__name__)

class DebugConfigProxy:
    """
    A proxy class that provides debug configuration with dynamic updates
    and runtime configuration capabilities.
    """
    
    def __init__(self, config_source):
        self._config_source = config_source
        self._cached_attrs = {}
        self._is_fallback = hasattr(config_source, '_is_fallback_config')
        
    def __getattr__(self, name):
        """Get debug flag value with caching and dynamic lookup."""
        if name in self._cached_attrs:
            return self._cached_attrs[name]
            
        try:
            value = getattr(self._config_source, name)
            self._cached_attrs[name] = value
            return value
        except AttributeError:
            # Return a safe default for unknown debug flags
            logger.warning(f"Debug flag '{name}' not found, defaulting to False")
            self._cached_attrs[name] = False
            return False
            
    def __setattr__(self, name, value):
        """Set debug flag value with propagation to source."""
        if name.startswith('_'):
            super().__setattr__(name, value)
            return
            
        # Update both cache and source
        self._cached_attrs[name] = value
        try:
            setattr(self._config_source, name, value)
        except AttributeError:
            # If source doesn't support setting, just cache it
            pass
            
    def is_fallback(self):
        """Check if this is using fallback configuration."""
        return self._is_fallback
        
    def reload_config(self):
        """Clear cache to force reload of configuration values."""
        self._cached_attrs.clear()

def import_debug_config():
    """
    Safely import debug_config with multiple fallback strategies.
    
    Returns:
        DebugConfigProxy: A proxy object that provides debug configuration
    """
    # Strategy 1: Try direct import (works in normal development)
    try:
        import debug_config  # type: ignore
        logger.info("Successfully imported debug_config directly")
        return DebugConfigProxy(debug_config)
    except ImportError:
        logger.debug("Direct import of debug_config failed, trying path-based import")
    
    # Strategy 2: Try adding debug path and importing
    try:
        # Get the project root directory
        current_file = Path(__file__).resolve()
        project_root = current_file.parent.parent  # Go up to scara directory
        debug_path = project_root / 'debug'
        
        if debug_path.exists() and str(debug_path) not in sys.path:
            sys.path.insert(0, str(debug_path))
            logger.debug(f"Added debug path to sys.path: {debug_path}")
        
        import debug_config  # type: ignore
        logger.info("Successfully imported debug_config via path manipulation")
        return DebugConfigProxy(debug_config)
    except ImportError:
        logger.debug("Path-based import of debug_config failed, trying fallback module")
    
    # Strategy 3: Use built-in fallback
    try:
        from .debug_fallback import DebugConfig
        
        # Create a module-like object with fallback marker
        class DebugModule:
            def __init__(self):
                self._is_fallback_config = True
                config = DebugConfig.get_instance()
                # Copy all attributes from DebugConfig instance
                for attr in dir(config):
                    if not attr.startswith('_') and not callable(getattr(config, attr)):
                        setattr(self, attr, getattr(config, attr))
        
        logger.info("Using built-in debug fallback configuration")
        return DebugConfigProxy(DebugModule())
    except ImportError:
        logger.warning("Built-in fallback import failed, using last resort fallback")
    
    # Strategy 4: Last resort fallback
    class FallbackDebugConfig:
        def __init__(self):
            self._is_fallback_config = True
            # Essential debug flags with safe defaults
            self.DEBUG_KEY_EVENT = True
            self.DEBUG_IMGUI_CAPTURE = True
            self.DEBUG_ACTION_IGNORE = True
            self.DEBUG_MANUAL_MODE_OFF = True
            self.DEBUG_KEY_W = True
            self.DEBUG_KEY_S = True
            self.DEBUG_KEY_A = True
            self.DEBUG_KEY_D = True
            self.DEBUG_KEY_Q = True
            self.DEBUG_KEY_E = True
            self.DEBUG_KEY_R = True
            self.DEBUG_KEY_F = True
            self.DEBUG_MANUAL_RPMS = False
            self.DEBUG_PHYSICS = False
            self.DEBUG_MANUAL_STATUS = False
    
    logger.info("Using last resort debug configuration")
    return DebugConfigProxy(FallbackDebugConfig())

# Create global instance with lazy initialization
_debug_config_instance = None

def get_debug_config():
    """Get the global debug configuration instance."""
    global _debug_config_instance
    if _debug_config_instance is None:
        _debug_config_instance = import_debug_config()
    return _debug_config_instance

def reload_debug_config():
    """Force reload of debug configuration."""
    global _debug_config_instance
    _debug_config_instance = None
    return get_debug_config()

# Global instance for easy access
debug_config = get_debug_config()
