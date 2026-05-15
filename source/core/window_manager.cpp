//
// Created by orange on 16.02.2026.
//
#ifdef _WIN32
#include <windows.h>
#include <timeapi.h>
#endif

#include "rose/core/window_manager.hpp"
#include "rose/core/collision_world.hpp"
#include "rose/core/model.hpp"
#include "rose/core/player.hpp"
#include "rose/core/vulkan/renderer.hpp"
#include "rose/plugins/plugin_sdk.hpp"

#include <GLFW/glfw3.h>
#include <boost/dll.hpp>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include <omath/engines/opengl_engine/camera.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <thread>

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

        if (glfwVulkanSupported() != GLFW_TRUE)
        {
            spdlog::critical("GLFW reports that Vulkan is not supported.");
            std::terminate();
        }

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        m_window = glfwCreateWindow(m_window_size.x, m_window_size.y, "ROSE", nullptr, nullptr);
        if (m_window == nullptr)
        {
            spdlog::critical("Failed to create Window!");
            std::terminate();
        }

#ifdef _WIN32
        timeBeginPeriod(1);
#endif

        ImGui::CreateContext();
        m_renderer = std::make_unique<vulkan::Renderer>(m_window, m_window_size);
    }

    WindowManager::~WindowManager()
    {
        m_renderer.reset();
        if (ImGui::GetCurrentContext() != nullptr)
            ImGui::DestroyContext();

        if (m_window != nullptr)
        {
            glfwDestroyWindow(m_window);
            m_window = nullptr;
        }
        glfwTerminate();

#ifdef _WIN32
        timeEndPeriod(1);
#endif
    }

    void WindowManager::run()
    {
        const boost::dll::fs::path lib_path("rose.stream.dll");
        using pluginapi_create_t = std::shared_ptr<StreamPluginApi>();
        auto creator = boost::dll::import_alias<pluginapi_create_t>(
            lib_path,
            "create_plugin",
            boost::dll::load_mode::default_mode
        );
        std::shared_ptr<StreamPluginApi> plugin = creator();
        plugin->run();

        auto map = Model("map2.glb");

        spdlog::info("Building {} map colliders...", map.get_meshes().size());
        std::vector<CollisionWorld::Collider> raw_colliders;
        raw_colliders.reserve(map.get_meshes().size());
        for (const auto& mesh : map.get_meshes())
            raw_colliders.emplace_back(mesh.cpu_mesh());
        const auto world = CollisionWorld::build(std::move(raw_colliders));
        spdlog::info("Collision world ready ({} colliders, chunk size {:.0f} m).",
                     world.colliders.size(), CollisionWorld::k_chunk_size);

        Player player{{0.f, 5.f, 0.f}};

        auto framebuffer = m_renderer->framebuffer_size();
        omath::opengl_engine::Camera camera{
            player.get_eye_position(),
            player.get_view_angles(),
            {static_cast<float>(framebuffer.x), static_cast<float>(framebuffer.y)},
            omath::projection::FieldOfView::from_degrees(90.f),
            0.1f,
            10000.f
        };

        bool   mouse_captured  = false;
        bool   esc_was_pressed = false;
        bool   first_mouse  = true;
        double last_mouse_x = 0.0;
        double last_mouse_y = 0.0;
        double last_time    = glfwGetTime();

        while (!glfwWindowShouldClose(m_window))
        {
            glfwPollEvents();

            const double current_time = glfwGetTime();
            const float delta_time = std::min(static_cast<float>(current_time - last_time), 0.05f);
            last_time = current_time;

            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            ImGui::Begin("Performance");
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                        1000.0 / ImGui::GetIO().Framerate,
                        ImGui::GetIO().Framerate);
            ImGui::End();

            const bool esc_pressed = glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
            if (esc_pressed && !esc_was_pressed)
            {
                mouse_captured = !mouse_captured;
                glfwSetInputMode(m_window, GLFW_CURSOR,
                                 mouse_captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
                first_mouse = true;
            }
            esc_was_pressed = esc_pressed;

            PlayerInput input;
            input.forward  = glfwGetKey(m_window, GLFW_KEY_W)     == GLFW_PRESS;
            input.backward = glfwGetKey(m_window, GLFW_KEY_S)     == GLFW_PRESS;
            input.right    = glfwGetKey(m_window, GLFW_KEY_D)     == GLFW_PRESS;
            input.left     = glfwGetKey(m_window, GLFW_KEY_A)     == GLFW_PRESS;
            input.jump     = glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS;
            input.noclip   = glfwGetKey(m_window, GLFW_KEY_Q)     == GLFW_PRESS;

            if (mouse_captured)
            {
                double mx = 0.0;
                double my = 0.0;
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

            ImGui::Render();

            if (m_renderer->begin_frame())
            {
                map.draw(*m_renderer, camera);
                m_renderer->render_imgui(ImGui::GetDrawData());

                const bool capture_frame = plugin->is_ready_to_stream();
                auto bmp_data = m_renderer->end_frame(capture_frame);
                if (capture_frame && !bmp_data.empty())
                    plugin->push_frame(bmp_data);
            }

            framebuffer = m_renderer->framebuffer_size();
            camera.set_view_port({static_cast<float>(framebuffer.x), static_cast<float>(framebuffer.y)});

            static constexpr double k_target_frame_time = 1.0 / 60.0;
            const double frame_target = current_time + k_target_frame_time;
            const double sleep_s = frame_target - 0.002 - glfwGetTime();
            if (sleep_s > 0.0)
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
            while (glfwGetTime() < frame_target)
                ;
        }

        m_renderer->wait_idle();
    }
} // namespace rose::core
