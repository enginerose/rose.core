//
// Created by orange on 16.02.2026.
//
#include <GL/glew.h>
#include "rose/core/window_manager.hpp"
#include "rose/core/opengl/shader_program.hpp"
#include <spdlog/spdlog.h>

static void GlfwErrorCallback(int code, const char* desc)
{
    spdlog::error("GLFW error {}: {}", code, desc);
}

namespace rose::core
{
    WindowManager::WindowManager()
    {
        glfwSetErrorCallback(GlfwErrorCallback);
        if (!glfwInit())
        {
            spdlog::critical("Failed to initialize GLFW!");
            std::terminate();
        }
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);


        m_window = glfwCreateWindow(m_window_size.x, m_window_size.y, "ROSE", nullptr, nullptr);
        if (!m_window)
        {
            spdlog::critical("Failed to create Window!");
            std::terminate();
        }
        glfwMakeContextCurrent(m_window);
        GLenum glewErr = glewInit();
        if (glewErr != GLEW_OK)
        {
            spdlog::critical("Failed to initialize GLEW, reason {}", reinterpret_cast<const char*>(glewGetErrorString(glewErr)));
            std::terminate();
        }
    }
    void WindowManager::run()
    {
        auto shader_program = opengl::ShaderProgram::from_files("shaders/shader.vert", "shaders/shader.frag");

        shader_program.use();
        while (true)
        {
            shader_program.use();
            glfwMakeContextCurrent(m_window);
            glClear(GL_COLOR_BUFFER_BIT);
            glfwSwapBuffers(m_window);

            shader_program.use();
            if (glfwWindowShouldClose(m_window) || glfwGetKey(m_window, GLFW_KEY_ESCAPE))
            {
                glfwTerminate();
                exit(EXIT_SUCCESS);
            }

            glfwWaitEvents();
            glfwMakeContextCurrent(m_window);
            glClearColor(0.3f, 0.3f, 0.3f, 1.f);
        }
    }
} // namespace rose::core