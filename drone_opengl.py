"""
Quadcopter PyOpenGL Simulation - Main Module
"""
import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
import time
from quadcopter_sim.simulation import QuadcopterSimulation
from quadcopter_sim.renderer import Renderer

def main():
    sim = QuadcopterSimulation()
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
