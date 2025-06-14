"""
Quadcopter PyOpenGL Simulation - Main Module
"""
import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
import time
import numpy as np
from quadcopter_sim.simulation import QuadcopterSimulation
from quadcopter_sim.renderer import Renderer

def key_callback(window, key, scancode, action, mods):
    # Import key constants
    import glfw
    import imgui
    # Use global sim for access
    global sim
    # Prevent handling if ImGui wants to capture keyboard
    if imgui.get_io().want_capture_keyboard:
        return
    # Only act on press or repeat
    if action not in (glfw.PRESS, glfw.REPEAT):
        return
    # Toggle manual mode
    if key == glfw.KEY_M:
        sim.manual_mode = not sim.manual_mode
        if sim.manual_mode:
            sim.set_manual_hover()
        print(f"Manual mode: {sim.manual_mode}")
    # Only process manual controls if in manual mode
    if not sim.manual_mode:
        return
    # Rotor mapping: [FL, FR, RR, RL]
    # W/S: pitch, A/D: roll, Q/E: yaw, R/F: throttle
    delta = 100  # RPM step
    if key == glfw.KEY_W:
        sim.manual_rpms[0] += delta
        sim.manual_rpms[1] += delta
        sim.manual_rpms[2] -= delta
        sim.manual_rpms[3] -= delta
    elif key == glfw.KEY_S:
        sim.manual_rpms[0] -= delta
        sim.manual_rpms[1] -= delta
        sim.manual_rpms[2] += delta
        sim.manual_rpms[3] += delta
    elif key == glfw.KEY_A:
        sim.manual_rpms[0] += delta
        sim.manual_rpms[3] += delta
        sim.manual_rpms[1] -= delta
        sim.manual_rpms[2] -= delta
    elif key == glfw.KEY_D:
        sim.manual_rpms[0] -= delta
        sim.manual_rpms[3] -= delta
        sim.manual_rpms[1] += delta
        sim.manual_rpms[2] += delta
    elif key == glfw.KEY_Q:
        sim.manual_rpms[0] += delta
        sim.manual_rpms[2] += delta
        sim.manual_rpms[1] -= delta
        sim.manual_rpms[3] -= delta
    elif key == glfw.KEY_E:
        sim.manual_rpms[0] -= delta
        sim.manual_rpms[2] -= delta
        sim.manual_rpms[1] += delta
        sim.manual_rpms[3] += delta
    elif key == glfw.KEY_R:
        sim.manual_rpms += delta
    elif key == glfw.KEY_F:
        sim.manual_rpms -= delta
    # Clamp RPMs
    sim.manual_rpms = np.clip(sim.manual_rpms, sim.min_rpm, sim.max_rpm)

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
    glfw.set_key_callback(window, key_callback)
    from OpenGL.GL import glEnable, GL_DEPTH_TEST, glClearColor
    glEnable(GL_DEPTH_TEST)
    glClearColor(1, 1, 1, 1)
    imgui.create_context()
    impl = GlfwRenderer(window)
    last_time = time.time()
    while not glfw.window_should_close(window):
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time
        glfw.poll_events()
        impl.process_inputs()
        renderer.handle_mouse(window)
        width, height = glfw.get_framebuffer_size(window)
        renderer.reshape(width, height)
        imgui.new_frame()
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
