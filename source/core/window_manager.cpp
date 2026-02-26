//
// Created by orange on 16.02.2026.
//
#include "rose/core/model.hpp"
#include "rose/core/window_manager.hpp"
#include "rose/core/opengl/shader_program.hpp"
#include <GL/glew.h>
#include <cmath>
#include <omath/engines/opengl_engine/camera.hpp>
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
            spdlog::critical(
                    "Failed to initialize GLEW, reason {}", reinterpret_cast<const char*>(glewGetErrorString(glewErr))
            );
            std::terminate();
        }
    }
    void WindowManager::run() const
    {
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.3f, 0.3f, 0.3f, 1.f);

        const auto model = Model("/home/orange/Downloads/buggie.glb");

        constexpr float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

        omath::opengl_engine::Camera camera{
                {0, 0, 200}, {}, {800.f, 800.f}, omath::projection::FieldOfView::from_degrees(90.f), 0.1f, 1000.f
        };

        constexpr float move_speed = 100.f;
        constexpr float mouse_sensitivity = 0.1f;

        bool mouse_captured = false;
        bool esc_was_pressed = false;
        bool first_mouse = true;
        double last_mouse_x = 0.0, last_mouse_y = 0.0;
        double last_time = glfwGetTime();

        while (true)
        {
            glfwPollEvents();

            if (glfwWindowShouldClose(m_window))
            {
                glfwTerminate();
                exit(EXIT_SUCCESS);
            }

            const double current_time = glfwGetTime();
            const float delta_time = static_cast<float>(current_time - last_time);
            last_time = current_time;

            // ESC toggles mouse capture
            const bool esc_pressed = glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
            if (esc_pressed && !esc_was_pressed)
            {
                mouse_captured = !mouse_captured;
                glfwSetInputMode(m_window, GLFW_CURSOR, mouse_captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
                first_mouse = true;
            }
            esc_was_pressed = esc_pressed;

            // Mouse look
            if (mouse_captured)
            {
                double mouse_x, mouse_y;
                glfwGetCursorPos(m_window, &mouse_x, &mouse_y);
                if (first_mouse)
                {
                    last_mouse_x = mouse_x;
                    last_mouse_y = mouse_y;
                    first_mouse = false;
                }
                const float dx = static_cast<float>(mouse_x - last_mouse_x) * mouse_sensitivity;
                const float dy = static_cast<float>(mouse_y - last_mouse_y) * mouse_sensitivity;
                last_mouse_x = mouse_x;
                last_mouse_y = mouse_y;

                auto angles = camera.get_view_angles();
                angles.yaw -= decltype(angles.yaw)::from_degrees(dx);
                angles.pitch -= decltype(angles.pitch)::from_degrees(dy); // invert Y so mouse-up = look up
                camera.set_view_angles(angles);
            }

            // WASD + Space/Ctrl movement
            {
                const float yaw_rad = camera.get_view_angles().yaw.as_radians();
                const omath::Vector3<float> forward = {-std::sin(yaw_rad), 0.f, -std::cos(yaw_rad)};
                const omath::Vector3<float> right = {std::cos(yaw_rad), 0.f, -std::sin(yaw_rad)};

                omath::Vector3<float> move = {0.f, 0.f, 0.f};
                if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
                    move = move + forward;
                if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS)
                    move = move - forward;
                if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS)
                    move = move + right;
                if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS)
                    move = move - right;
                if (glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS)
                    move.y += 1.f;
                if (glfwGetKey(m_window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
                    move.y -= 1.f;

                const float speed = move_speed * delta_time;
                camera.set_origin(
                        camera.get_origin() + omath::Vector3<float>{move.x * speed, move.y * speed, move.z * speed}
                );
            }

            int fb_w, fb_h;
            glfwGetFramebufferSize(m_window, &fb_w, &fb_h);
            glViewport(0, 0, fb_w, fb_h);
            camera.set_view_port({static_cast<float>(fb_w), static_cast<float>(fb_h)});

            auto shader_program = opengl::ShaderProgram::from_files("shaders/shader.vert", "shaders/shader.frag");
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            shader_program.use();
            shader_program.set_mat4("uMVP", camera.get_view_projection_matrix().raw_array().data());
            shader_program.set_mat4("uModel", identity);
            model.draw(shader_program);

            glfwSwapBuffers(m_window);
        }
    }
} // namespace rose::core