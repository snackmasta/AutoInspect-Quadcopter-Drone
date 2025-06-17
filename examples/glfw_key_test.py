import glfw

def key_callback(window, key, scancode, action, mods):
    print(f"[GLFW TEST] key={key}, action={action}, scancode={scancode}, mods={mods}")

def main():
    if not glfw.init():
        print("Could not initialize GLFW")
        return
    window = glfw.create_window(400, 300, "GLFW Key Test", None, None)
    if not window:
        glfw.terminate()
        print("Could not create GLFW window")
        return
    glfw.make_context_current(window)
    glfw.set_key_callback(window, key_callback)
    print("[GLFW TEST] Window created and key_callback registered.")
    while not glfw.window_should_close(window):
        glfw.poll_events()
    glfw.terminate()

if __name__ == "__main__":
    main()
