import sys
import os

# Add the debug directory to sys.path so debug_config can be imported
if hasattr(sys, '_MEIPASS'):
    # Running as PyInstaller bundle
    debug_path = os.path.join(sys._MEIPASS, 'debug')
    if debug_path not in sys.path:
        sys.path.insert(0, debug_path)
    
    # Also add debug modules to sys.modules to make them importable
    import importlib.util
    
    # Import debug_config directly and add to sys.modules
    debug_config_path = os.path.join(debug_path, 'debug_config.py')
    if os.path.exists(debug_config_path):
        spec = importlib.util.spec_from_file_location("debug_config", debug_config_path)
        debug_config = importlib.util.module_from_spec(spec)
        sys.modules['debug_config'] = debug_config
        spec.loader.exec_module(debug_config)
    
    # Import debug_physics and add to sys.modules if it exists
    debug_physics_path = os.path.join(debug_path, 'debug_physics.py')
    if os.path.exists(debug_physics_path):
        spec = importlib.util.spec_from_file_location("debug_physics", debug_physics_path)
        debug_physics = importlib.util.module_from_spec(spec)
        sys.modules['debug_physics'] = debug_physics
        spec.loader.exec_module(debug_physics)
