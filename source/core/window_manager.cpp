//
// Created by orange on 16.02.2026.
//
#include <GL/glew.h>
#include "rose/core/window_manager.hpp"
#include "/home/orange/CLionProjects/rose.stream_plugin/include/cherry_streamer/stream_plugin_api.hpp"
#include "rose/core/collision_world.hpp"
#include "rose/core/model.hpp"
#include "rose/core/opengl/screenshot.hpp"
#include "rose/core/opengl/shader_program.hpp"
#include "rose/core/player.hpp"
#include "rose/core/thread_pool.hpp"
#include <boost/dll.hpp>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
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

        glfwSwapInterval(0);
        ImGui::CreateContext();
        ImGui_ImplGlfw_InitForOpenGL(m_window, true);
        ImGui_ImplOpenGL3_Init("#version 330");

    }

    void WindowManager::run() const
    {
        const boost::dll::fs::path lib_path(
            "/home/orange/CLionProjects/rose.core/plugins/rose.stream.so"
    ); // argv[1] contains path to directory with our plugin library
        using pluginapi_create_t = std::shared_ptr<StreamPluginApi>();
        auto creator = boost::dll::import_alias<pluginapi_create_t>(        // type of imported symbol must be explicitly specified
            lib_path,                                            // path to library
            "create_plugin",                                                // symbol to import
            boost::dll::load_mode::default_mode                              // do append extensions and prefixes
        );
        std::shared_ptr<StreamPluginApi> plugin = creator();
        plugin->run();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        glClearColor(0.3f, 0.3f, 0.3f, 1.f);

        auto map = Model("/home/orange/Downloads/map2.glb");

        // Build one MeshCollider per map mesh, then index them into a spatial chunk grid.
        spdlog::info("Building {} map colliders...", map.get_meshes().size());
        std::vector<CollisionWorld::Collider> raw_colliders;
        raw_colliders.reserve(map.get_meshes().size());
        for (const auto& mesh : map.get_meshes())
            raw_colliders.emplace_back(mesh.cpu_mesh());
        const auto world = CollisionWorld::build(std::move(raw_colliders));
        spdlog::info("Collision world ready ({} colliders, chunk size {:.0f} m).",
                     world.colliders.size(), CollisionWorld::k_chunk_size);

        ThreadPool thread_pool{};
        Player player{{0.f, 5.f, 0.f}, thread_pool};

        omath::opengl_engine::Camera camera{
            player.get_eye_position(),
            player.get_view_angles(),
            {static_cast<float>(m_window_size.x), static_cast<float>(m_window_size.y)},
            omath::projection::FieldOfView::from_degrees(90.f),
            0.1f,
            10000.f
        };

        auto shader_program = opengl::ShaderProgram::from_files("shaders/shader.vert", "shaders/shader.frag");

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
            const float delta_time = std::min(static_cast<float>(current_time - last_time), 0.05f);
            last_time = current_time;

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // Your other ImGui windows and widgets here...

            // Example of how to display the FPS in a window
            ImGui::Begin("Performance");
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0 / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
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
            player.update(delta_time, world, input);
            camera.set_origin(player.get_eye_position());
            camera.set_view_angles(player.get_view_angles());
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            shader_program.use();
            shader_program.set_mat4("uMVP", camera.get_view_projection_matrix().raw_array().data());
            map.draw(shader_program, camera, thread_pool);
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            int fb_w, fb_h;
            glfwGetFramebufferSize(m_window, &fb_w, &fb_h);
            glViewport(0, 0, fb_w, fb_h);
            glfwSwapBuffers(m_window);

            if (plugin->is_ready_to_stream())
            {
                auto bmp_data = rose::core::take_screenshot(fb_w, fb_h);
                plugin->push_frame(bmp_data);
            }
            camera.set_view_port({static_cast<float>(fb_w), static_cast<float>(fb_h)});

            // Precise 60 fps cap: sleep most of the budget, then spinwait the tail.
            static constexpr double k_target_frame_time = 1.0 / 60.0;
            const double frame_end_target = current_time + k_target_frame_time;
            const double sleep_until      = frame_end_target - 0.002; // leave 2 ms for spinwait
            const double now              = glfwGetTime();
            if (now < sleep_until)
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_until - now));
            while (glfwGetTime() < frame_end_target) {}
        }
    }
} // namespace rose::core
