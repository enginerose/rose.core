//
// Created by orange on 16.02.2026.
//
#include "rose/core/model.hpp"
#include "rose/core/player.hpp"
#include "rose/core/window_manager.hpp"
#include "rose/core/opengl/shader_program.hpp"
#include <GL/glew.h>
#include <omath/collision/mesh_collider.hpp>
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

        auto map = Model("/home/orange/Downloads/map.glb");

        // Build one MeshCollider per map mesh (copies CPU vertex data once at load time)
        spdlog::info("Building {} map colliders...", map.get_meshes().size());
        std::vector<omath::collision::MeshCollider<omath::opengl_engine::Mesh>> map_colliders;
        map_colliders.reserve(map.get_meshes().size());
        for (const auto& mesh : map.get_meshes())
            map_colliders.emplace_back(mesh.cpu_mesh());
        spdlog::info("Map colliders ready.");

        Player player{{0.f, 300.f, 0.f}};

        omath::opengl_engine::Camera camera{
            player.get_eye_position(),
            player.get_view_angles(),
            {static_cast<float>(m_window_size.x), static_cast<float>(m_window_size.y)},
            omath::projection::FieldOfView::from_degrees(90.f),
            0.1f,
            10000.f
        };

        bool   mouse_captured  = false;
        bool   esc_was_pressed = false;
        bool   first_mouse     = true;
        double last_mouse_x    = 0.0;
        double last_mouse_y    = 0.0;
        double last_time       = glfwGetTime();

        while (true)
        {
            glfwPollEvents();

            if (glfwWindowShouldClose(m_window))
            {
                glfwTerminate();
                exit(EXIT_SUCCESS);
            }

            const double current_time = glfwGetTime();
            const float  delta_time   = static_cast<float>(current_time - last_time);
            last_time = current_time;

            // --- ESC toggles mouse capture ---
            const bool esc_pressed = glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
            if (esc_pressed && !esc_was_pressed)
            {
                mouse_captured = !mouse_captured;
                glfwSetInputMode(m_window, GLFW_CURSOR,
                                 mouse_captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
                first_mouse = true;
            }
            esc_was_pressed = esc_pressed;

            // --- Collect input ---
            PlayerInput input;
            input.forward  = glfwGetKey(m_window, GLFW_KEY_W)     == GLFW_PRESS;
            input.backward = glfwGetKey(m_window, GLFW_KEY_S)     == GLFW_PRESS;
            input.right    = glfwGetKey(m_window, GLFW_KEY_D)     == GLFW_PRESS;
            input.left     = glfwGetKey(m_window, GLFW_KEY_A)     == GLFW_PRESS;
            input.jump     = glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS;

            if (mouse_captured)
            {
                double mx, my;
                glfwGetCursorPos(m_window, &mx, &my);
                if (!first_mouse)
                {
                    input.mouse_dx = static_cast<float>(mx - last_mouse_x);
                    input.mouse_dy = static_cast<float>(my - last_mouse_y);
                }
                else
                {
                    first_mouse = false;
                }
                last_mouse_x = mx;
                last_mouse_y = my;
            }

            // --- Update player (movement + gravity + GJK/EPA collision) ---
            player.update(delta_time, map_colliders, input);

            // --- Sync camera to player eye ---
            camera.set_origin(player.get_eye_position());
            camera.set_view_angles(player.get_view_angles());

            // --- Render ---
            int fb_w, fb_h;
            glfwGetFramebufferSize(m_window, &fb_w, &fb_h);
            glViewport(0, 0, fb_w, fb_h);
            camera.set_view_port({static_cast<float>(fb_w), static_cast<float>(fb_h)});

            auto shader_program = opengl::ShaderProgram::from_files("shaders/shader.vert", "shaders/shader.frag");
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            shader_program.use();
            shader_program.set_mat4("uMVP", camera.get_view_projection_matrix().raw_array().data());
            map.draw(shader_program);

            glfwSwapBuffers(m_window);
        }
    }
} // namespace rose::core
