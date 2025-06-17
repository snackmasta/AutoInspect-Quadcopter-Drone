"""
Quadcopter PyOpenGL Simulation - Main Module
"""
import sys
import os

# Add parent directory to path so we can import quadcopter_sim
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
import time
import numpy as np
from quadcopter_sim.simulation import QuadcopterSimulation
from quadcopter_sim.renderer import Renderer

# Import debug configuration using centralized utility
from quadcopter_sim.debug_utils import debug_config

def key_callback(window, key, scancode, action, mods):
    print("[DEBUG] key_callback entered")
    # Import key constants
    import glfw
    import imgui
    global sim
    if debug_config.DEBUG_KEY_EVENT:
        print(f"[DEBUG] key_callback fired: key={key}, action={action}, manual_mode={getattr(sim, 'manual_mode', None)}")
    # Prevent handling if ImGui wants to capture keyboard
    if imgui.get_io().want_capture_keyboard:
        if debug_config.DEBUG_IMGUI_CAPTURE:
            print("[DEBUG] ImGui is capturing keyboard input.")
        return
    # Only act on press or repeat
    if action not in (glfw.PRESS, glfw.REPEAT):
        if debug_config.DEBUG_ACTION_IGNORE:
            print(f"[DEBUG] Ignoring action: {action}")
        return    # Toggle manual mode
    if key == glfw.KEY_M:
        sim.manual_mode = not sim.manual_mode
        if sim.manual_mode:
            # When entering manual mode, preserve current position and set RPMs to current rotor speeds
            sim.manual_rpms[:] = sim.rotor_speeds
            print(f"[DEBUG] Entered manual mode - preserved position and set RPMs to current rotor speeds")
        else:
            # When exiting manual mode, reset manual RPMs and clean up manual state
            sim.manual_rpms[:] = 0
            if hasattr(sim, 'manual_yaw'):
                delattr(sim, 'manual_yaw')
            print(f"[DEBUG] Exited manual mode - reset manual RPMs")
        print(f"Manual mode: {sim.manual_mode}")
    # Only process manual controls if in manual mode
    if not sim.manual_mode:
        if debug_config.DEBUG_MANUAL_MODE_OFF:
            print("[DEBUG] Manual mode is OFF. Ignoring key input.")
        return
    # Rotor mapping: [FL, FR, RR, RL]
    # W/S: pitch, A/D: roll, Q/E: yaw, R/F: throttle
    delta = 100  # RPM step
    if key == glfw.KEY_W:
        if debug_config.DEBUG_KEY_W:
            print("[DEBUG] W pressed: pitch Forward")
        sim.manual_rpms[0] -= delta
        sim.manual_rpms[1] -= delta
        sim.manual_rpms[2] += delta
        sim.manual_rpms[3] += delta
    elif key == glfw.KEY_S:
        if debug_config.DEBUG_KEY_S:
            print("[DEBUG] S pressed: pitch Backward")
        sim.manual_rpms[0] += delta
        sim.manual_rpms[1] += delta
        sim.manual_rpms[2] -= delta
        sim.manual_rpms[3] -= delta
    elif key == glfw.KEY_A:
        if debug_config.DEBUG_KEY_A:
            print("[DEBUG] A pressed: roll left")
        sim.manual_rpms[0] += delta
        sim.manual_rpms[3] += delta
        sim.manual_rpms[1] -= delta
        sim.manual_rpms[2] -= delta
    elif key == glfw.KEY_D:
        if debug_config.DEBUG_KEY_D:
            print("[DEBUG] D pressed: roll right")
        sim.manual_rpms[0] -= delta
        sim.manual_rpms[3] -= delta
        sim.manual_rpms[1] += delta
        sim.manual_rpms[2] += delta
    elif key == glfw.KEY_Q:
        if debug_config.DEBUG_KEY_Q:
            print("[DEBUG] Q pressed: yaw left")
        if not hasattr(sim, 'manual_yaw'):
            sim.manual_yaw = 0.0
        sim.manual_yaw += np.radians(2)  # increment yaw by 2 degrees
    elif key == glfw.KEY_E:
        if debug_config.DEBUG_KEY_E:
            print("[DEBUG] E pressed: yaw right")
        if not hasattr(sim, 'manual_yaw'):
            sim.manual_yaw = 0.0
        sim.manual_yaw -= np.radians(2)  # decrement yaw by 2 degrees
    elif key == glfw.KEY_R:
        if debug_config.DEBUG_KEY_R:
            print("[DEBUG] R pressed: throttle up")
        sim.manual_rpms += delta
    elif key == glfw.KEY_F:
        if debug_config.DEBUG_KEY_F:
            print("[DEBUG] F pressed: throttle down")
        sim.manual_rpms -= delta
    elif key == glfw.KEY_SPACE:
        if debug_config.DEBUG_KEY_EVENT:
            print("[DEBUG] SPACE pressed: set all rotors to 0 RPM")
        sim.manual_rpms[:] = 0
        sim.rotor_speeds[:] = 0
    elif key == glfw.KEY_V:
        if debug_config.DEBUG_KEY_EVENT:
            print("[DEBUG] V pressed: reset all velocities to 0")
        sim.state[3:6] = 0  # vx, vy, vz
    # Clamp RPMs
    sim.manual_rpms = np.clip(sim.manual_rpms, sim.min_rpm, sim.max_rpm)
    if debug_config.DEBUG_MANUAL_RPMS:
        print(f"[DEBUG] manual_rpms: {sim.manual_rpms}")
    if hasattr(sim, 'manual_yaw'):
        if debug_config.DEBUG_MANUAL_RPMS:
            print(f"[DEBUG] manual_yaw: {np.degrees(sim.manual_yaw):.2f} deg")

def mouse_button_callback(window, button, action, mods):
    print(f"[DEBUG] mouse_button_callback: button={button}, action={action}")

def main():
    global sim
    sim = QuadcopterSimulation()
    sim.manual_mode = True
    sim.set_manual_hover(target_altitude=1.0)
    renderer = Renderer(sim)
    if not glfw.init():
        print("Could not initialize GLFW")
        return
    window = glfw.create_window(renderer.window_width, renderer.window_height, "Quadcopter PyOpenGL Simulation", None, None)
    if not window:
        glfw.terminate()
        print("Could not create GLFW window")
        return
    glfw.make_context_current(window)
    print("[DEBUG] About to register key_callback")
    glfw.set_mouse_button_callback(window, mouse_button_callback)
    print("[DEBUG] mouse_button_callback registered")
    from OpenGL.GL import glEnable, GL_DEPTH_TEST, glClearColor
    glEnable(GL_DEPTH_TEST)
    glClearColor(1, 1, 1, 1)
    imgui.create_context()
    impl = GlfwRenderer(window)
    last_time = time.time()
    print("[DEBUG] Registering key_callback just before main loop")
    glfw.set_key_callback(window, key_callback)
    print("[DEBUG] key_callback registered (late)")
    while not glfw.window_should_close(window):
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time
        glfw.poll_events()
        impl.process_inputs()  # TEMPORARILY DISABLED FOR DEBUGGING
        renderer.handle_mouse(window)
        width, height = glfw.get_framebuffer_size(window)
        renderer.reshape(width, height)
        imgui.new_frame()
        # Debug toggles window
        imgui.set_next_window_position(10, 520, imgui.ONCE)
        imgui.set_next_window_size(350, 0, imgui.ONCE)
        imgui.begin("Debug Toggles", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        _, debug_config.DEBUG_KEY_EVENT = imgui.checkbox("Key Event Debug", debug_config.DEBUG_KEY_EVENT)
        _, debug_config.DEBUG_IMGUI_CAPTURE = imgui.checkbox("ImGui Capture Debug", debug_config.DEBUG_IMGUI_CAPTURE)
        _, debug_config.DEBUG_ACTION_IGNORE = imgui.checkbox("Action Ignore Debug", debug_config.DEBUG_ACTION_IGNORE)
        _, debug_config.DEBUG_MANUAL_MODE_OFF = imgui.checkbox("Manual Mode Off Debug", debug_config.DEBUG_MANUAL_MODE_OFF)
        _, debug_config.DEBUG_KEY_W = imgui.checkbox("W Key Debug", debug_config.DEBUG_KEY_W)
        _, debug_config.DEBUG_KEY_S = imgui.checkbox("S Key Debug", debug_config.DEBUG_KEY_S)
        _, debug_config.DEBUG_KEY_A = imgui.checkbox("A Key Debug", debug_config.DEBUG_KEY_A)
        _, debug_config.DEBUG_KEY_D = imgui.checkbox("D Key Debug", debug_config.DEBUG_KEY_D)
        _, debug_config.DEBUG_KEY_Q = imgui.checkbox("Q Key Debug", debug_config.DEBUG_KEY_Q)
        _, debug_config.DEBUG_KEY_E = imgui.checkbox("E Key Debug", debug_config.DEBUG_KEY_E)
        _, debug_config.DEBUG_KEY_R = imgui.checkbox("R Key Debug", debug_config.DEBUG_KEY_R)
        _, debug_config.DEBUG_KEY_F = imgui.checkbox("F Key Debug", debug_config.DEBUG_KEY_F)
        _, debug_config.DEBUG_MANUAL_RPMS = imgui.checkbox("Manual RPMs Debug", debug_config.DEBUG_MANUAL_RPMS)
        _, debug_config.DEBUG_PHYSICS = imgui.checkbox("Physics Debug", debug_config.DEBUG_PHYSICS)
        _, debug_config.DEBUG_MANUAL_STATUS = imgui.checkbox("Manual Mode Status Debug", debug_config.DEBUG_MANUAL_STATUS)
        imgui.end()
        sim.step(delta_time)
        renderer.draw_scene()
        imgui.render()
        draw_data = imgui.get_draw_data()
        if draw_data is not None:
            impl.render(draw_data)
        glfw.swap_buffers(window)
    impl.shutdown()
    glfw.terminate()

if __name__ == "__main__":
    main()
