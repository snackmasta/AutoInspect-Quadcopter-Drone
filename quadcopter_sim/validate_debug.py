"""
Debug configuration validation and management tool.
Provides utilities to validate and manage debug configuration.
"""

import sys
import os
from pathlib import Path

def validate_debug_config():
    """Validate that debug configuration is working properly."""
    try:
        # Add parent directory to path
        sys.path.append(str(Path(__file__).parent.parent))
        
        from quadcopter_sim.debug_utils import debug_config, get_debug_config
        
        print("=== Debug Configuration Validation ===")
        
        # Check if config loaded successfully
        config_instance = get_debug_config()
        is_fallback = config_instance.is_fallback() if hasattr(config_instance, 'is_fallback') else True
        
        print(f"Configuration loaded: {'✓' if config_instance else '✗'}")
        print(f"Using fallback: {'Yes' if is_fallback else 'No'}")
        
        # Check essential debug flags
        essential_flags = [
            'DEBUG_KEY_EVENT',
            'DEBUG_PHYSICS',
            'DEBUG_MANUAL_STATUS',
            'DEBUG_KEY_W',
            'DEBUG_KEY_S',
            'DEBUG_KEY_A',
            'DEBUG_KEY_D'
        ]
        
        print("\n=== Essential Debug Flags ===")
        for flag in essential_flags:
            try:
                value = getattr(debug_config, flag)
                print(f"{flag}: {value}")
            except AttributeError:
                print(f"{flag}: MISSING ✗")
        
        # Test dynamic flag access
        print("\n=== Dynamic Flag Access Test ===")
        try:
            # Test getting a non-existent flag
            test_value = getattr(debug_config, 'DEBUG_NONEXISTENT_FLAG', 'DEFAULT')
            print(f"Non-existent flag handling: ✓ (got: {test_value})")
        except Exception as e:
            print(f"Non-existent flag handling: ✗ (error: {e})")
        
        # Test flag modification
        print("\n=== Flag Modification Test ===")
        try:
            original_value = debug_config.DEBUG_KEY_EVENT
            debug_config.DEBUG_KEY_EVENT = not original_value
            new_value = debug_config.DEBUG_KEY_EVENT
            debug_config.DEBUG_KEY_EVENT = original_value  # Restore
            print(f"Flag modification: ✓ (changed {original_value} → {new_value} → {original_value})")
        except Exception as e:
            print(f"Flag modification: ✗ (error: {e})")
        
        print("\n=== Validation Complete ===")
        
    except Exception as e:
        print(f"Validation failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    validate_debug_config()
