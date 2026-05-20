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
#include <ImGuizmo.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include <omath/engines/opengl_engine/camera.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <iterator>
#include <optional>
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
        auto world = CollisionWorld::build(std::move(raw_colliders));
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
        bool   overlay_open = false;
        bool   insert_was_pressed = false;
        bool   fullscreen = false;
        bool   f11_was_pressed = false;
        int    windowed_x = 0;
        int    windowed_y = 0;
        int    windowed_width = m_window_size.x;
        int    windowed_height = m_window_size.y;
        bool   restore_mouse_capture_after_overlay = false;
        bool   fps_cap_enabled = true;
        bool   auto_bhop = false;
        int    fps_limit = 60;
        std::optional<std::size_t> selected_mesh;
        ImGuizmo::OPERATION gizmo_operation = ImGuizmo::TRANSLATE;
        ImGuizmo::MODE      gizmo_mode = ImGuizmo::WORLD;
        bool   gizmo_snap_enabled = false;
        float  gizmo_translate_snap = 0.5f;
        float  gizmo_rotate_snap = 15.0f;
        float  gizmo_scale_snap = 0.1f;
        bool   first_mouse  = true;
        double last_mouse_x = 0.0;
        double last_mouse_y = 0.0;
        double last_time    = glfwGetTime();
        bool   left_mouse_was_pressed = false;
        bool   middle_mouse_was_pressed = false;

        const auto set_mouse_captured = [&](const bool captured)
        {
            mouse_captured = captured;
            glfwSetInputMode(m_window, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
            first_mouse = true;
        };

        const auto toggle_fullscreen = [&]
        {
            if (!fullscreen)
            {
                glfwGetWindowPos(m_window, &windowed_x, &windowed_y);
                glfwGetWindowSize(m_window, &windowed_width, &windowed_height);

                GLFWmonitor* monitor = glfwGetPrimaryMonitor();
                const GLFWvidmode* mode = monitor != nullptr ? glfwGetVideoMode(monitor) : nullptr;
                if (mode == nullptr)
                    return;

                glfwSetWindowMonitor(m_window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
                fullscreen = true;
            }
            else
            {
                glfwSetWindowMonitor(m_window, nullptr, windowed_x, windowed_y, windowed_width, windowed_height, 0);
                fullscreen = false;
            }
            first_mouse = true;
        };

        while (!glfwWindowShouldClose(m_window))
        {
            glfwPollEvents();

            const double current_time = glfwGetTime();
            const float delta_time = std::min(static_cast<float>(current_time - last_time), 0.05f);
            last_time = current_time;

            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            ImGuizmo::BeginFrame();

            ImDrawList* scene_gizmo_draw_list = nullptr;
            bool scene_gizmo_window_hovered = false;
            bool overlay_window_hovered = false;
            if (selected_mesh && !mouse_captured)
            {
                const ImGuiIO& io = ImGui::GetIO();
                ImGui::SetNextWindowPos({0.0f, 0.0f});
                ImGui::SetNextWindowSize(io.DisplaySize);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
                ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
                ImGui::Begin("SceneGizmo",
                             nullptr,
                             ImGuiWindowFlags_NoTitleBar
                             | ImGuiWindowFlags_NoResize
                             | ImGuiWindowFlags_NoMove
                             | ImGuiWindowFlags_NoScrollbar
                             | ImGuiWindowFlags_NoScrollWithMouse
                             | ImGuiWindowFlags_NoSavedSettings
                             | ImGuiWindowFlags_NoBackground
                             | ImGuiWindowFlags_NoBringToFrontOnFocus);
                scene_gizmo_draw_list = ImGui::GetWindowDrawList();
                scene_gizmo_window_hovered = ImGui::IsWindowHovered();
                ImGui::End();
                ImGui::PopStyleVar(2);
            }

            const bool insert_pressed = glfwGetKey(m_window, GLFW_KEY_INSERT) == GLFW_PRESS;
            if (insert_pressed && !insert_was_pressed)
            {
                overlay_open = !overlay_open;
                if (overlay_open)
                {
                    restore_mouse_capture_after_overlay = mouse_captured;
                    set_mouse_captured(false);
                }
                else if (restore_mouse_capture_after_overlay)
                {
                    set_mouse_captured(true);
                    restore_mouse_capture_after_overlay = false;
                }
            }
            insert_was_pressed = insert_pressed;

            const bool f11_pressed = glfwGetKey(m_window, GLFW_KEY_F11) == GLFW_PRESS;
            if (f11_pressed && !f11_was_pressed)
                toggle_fullscreen();
            f11_was_pressed = f11_pressed;

            if (overlay_open)
            {
                bool imgui_window_open = true;
                ImGui::SetNextWindowPos({16.f, 16.f}, ImGuiCond_FirstUseEver);
                ImGui::SetNextWindowSize({320.f, 0.f}, ImGuiCond_FirstUseEver);
                ImGui::Begin("ROSE Overlay", &imgui_window_open, ImGuiWindowFlags_AlwaysAutoResize);
                overlay_window_hovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows
                                                                | ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
                ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                            1000.0 / ImGui::GetIO().Framerate,
                            ImGui::GetIO().Framerate);
                ImGui::Separator();
                ImGui::Checkbox("FPS cap", &fps_cap_enabled);
                ImGui::BeginDisabled(!fps_cap_enabled);
                ImGui::SliderInt("FPS limit", &fps_limit, 30, 360);
                ImGui::EndDisabled();
                ImGui::Checkbox("Auto bhop", &auto_bhop);
                ImGui::Separator();

                bool dlss_enabled = m_renderer->dlss_enabled();
                ImGui::BeginDisabled(!m_renderer->dlss_available());
                if (ImGui::Checkbox("DLSS", &dlss_enabled))
                    m_renderer->set_dlss_enabled(dlss_enabled);

                constexpr const char* dlss_quality_labels[] = {
                    "Quality",
                    "Balanced",
                    "Performance",
                    "Ultra Performance",
                    "Ultra Quality",
                    "DLAA"
                };
                int dlss_quality = static_cast<int>(m_renderer->dlss_quality());
                if (ImGui::Combo("DLSS mode",
                                 &dlss_quality,
                                 dlss_quality_labels,
                                 static_cast<int>(std::size(dlss_quality_labels))))
                    m_renderer->set_dlss_quality(static_cast<vulkan::DlssQuality>(dlss_quality));
                ImGui::EndDisabled();
                ImGui::TextWrapped("%s", m_renderer->dlss_status().c_str());
                ImGui::Separator();
                if (selected_mesh)
                    ImGui::Text("Selected mesh: %zu", *selected_mesh);
                else
                    ImGui::TextUnformatted("Selected mesh: none");
                ImGui::BeginDisabled(!selected_mesh);
                if (ImGui::RadioButton("Move", gizmo_operation == ImGuizmo::TRANSLATE))
                    gizmo_operation = ImGuizmo::TRANSLATE;
                ImGui::SameLine();
                if (ImGui::RadioButton("Rotate", gizmo_operation == ImGuizmo::ROTATE))
                    gizmo_operation = ImGuizmo::ROTATE;
                ImGui::SameLine();
                if (ImGui::RadioButton("Scale", gizmo_operation == ImGuizmo::SCALE))
                    gizmo_operation = ImGuizmo::SCALE;

                if (ImGui::RadioButton("World", gizmo_mode == ImGuizmo::WORLD))
                    gizmo_mode = ImGuizmo::WORLD;
                ImGui::SameLine();
                if (ImGui::RadioButton("Local", gizmo_mode == ImGuizmo::LOCAL))
                    gizmo_mode = ImGuizmo::LOCAL;

                ImGui::Checkbox("Snap", &gizmo_snap_enabled);
                ImGui::BeginDisabled(!gizmo_snap_enabled);
                if (gizmo_operation == ImGuizmo::ROTATE)
                    ImGui::SliderFloat("Rotate snap", &gizmo_rotate_snap, 1.0f, 90.0f, "%.1f deg");
                else if (gizmo_operation == ImGuizmo::SCALE)
                    ImGui::SliderFloat("Scale snap", &gizmo_scale_snap, 0.01f, 1.0f, "%.2f");
                else
                    ImGui::SliderFloat("Move snap", &gizmo_translate_snap, 0.01f, 10.0f, "%.2f m");
                ImGui::EndDisabled();
                ImGui::EndDisabled();

                ImGui::Separator();
                auto outline_settings = m_renderer->selection_outline_settings();
                bool outline_settings_changed = false;
                outline_settings_changed |= ImGui::ColorEdit3("Glow color", outline_settings.color.data());
                outline_settings_changed |= ImGui::SliderFloat("Outline width",
                                                               &outline_settings.width,
                                                               0.01f,
                                                               0.5f,
                                                               "%.3f");
                outline_settings_changed |= ImGui::SliderInt("Smoothing quality",
                                                             &outline_settings.smoothing_quality,
                                                             1,
                                                             64);
                if (outline_settings_changed)
                    m_renderer->set_selection_outline_settings(outline_settings);

                ImGui::End();

                if (!imgui_window_open)
                    overlay_open = false;
            }

            if (!overlay_open && restore_mouse_capture_after_overlay)
            {
                set_mouse_captured(true);
                restore_mouse_capture_after_overlay = false;
            }

            const bool middle_mouse_pressed = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            if (!mouse_captured
                && selected_mesh
                && middle_mouse_pressed
                && !middle_mouse_was_pressed
                && !overlay_window_hovered
                && !ImGuizmo::IsUsing())
            {
                if (gizmo_operation == ImGuizmo::TRANSLATE)
                    gizmo_operation = ImGuizmo::ROTATE;
                else if (gizmo_operation == ImGuizmo::ROTATE)
                    gizmo_operation = ImGuizmo::SCALE;
                else
                    gizmo_operation = ImGuizmo::TRANSLATE;
            }
            middle_mouse_was_pressed = middle_mouse_pressed;

            const bool esc_pressed = glfwGetKey(m_window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
            if (!overlay_open && esc_pressed && !esc_was_pressed)
            {
                set_mouse_captured(!mouse_captured);
            }
            esc_was_pressed = esc_pressed;

            PlayerInput input;
            if (!overlay_open)
            {
                input.forward  = glfwGetKey(m_window, GLFW_KEY_W)     == GLFW_PRESS;
                input.backward = glfwGetKey(m_window, GLFW_KEY_S)     == GLFW_PRESS;
                input.right    = glfwGetKey(m_window, GLFW_KEY_D)     == GLFW_PRESS;
                input.left     = glfwGetKey(m_window, GLFW_KEY_A)     == GLFW_PRESS;
                input.jump     = glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS;
                input.auto_bhop = auto_bhop;
                input.noclip   = glfwGetKey(m_window, GLFW_KEY_Q)     == GLFW_PRESS;
            }

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
            framebuffer = m_renderer->framebuffer_size();
            camera.set_view_port({static_cast<float>(framebuffer.x), static_cast<float>(framebuffer.y)});

            if (selected_mesh && !mouse_captured && scene_gizmo_draw_list != nullptr)
            {
                if (auto model_matrix = map.mesh_matrix(*selected_mesh))
                {
                    const ImGuiIO& io = ImGui::GetIO();
                    auto gizmo_camera = camera;
                    gizmo_camera.set_view_port({io.DisplaySize.x, io.DisplaySize.y});

                    auto matrix = model_matrix->raw_array();
                    const auto view = gizmo_camera.get_view_matrix().raw_array();
                    const auto projection = gizmo_camera.get_projection_matrix().raw_array();

                    ImGuizmo::SetOrthographic(false);
                    ImGuizmo::SetDrawlist(scene_gizmo_draw_list);
                    ImGuizmo::SetRect(0.0f,
                                      0.0f,
                                      io.DisplaySize.x,
                                      io.DisplaySize.y);

                    std::array<float, 3> snap_values{};
                    if (gizmo_operation == ImGuizmo::ROTATE)
                        snap_values = {gizmo_rotate_snap, gizmo_rotate_snap, gizmo_rotate_snap};
                    else if (gizmo_operation == ImGuizmo::SCALE)
                        snap_values = {gizmo_scale_snap, gizmo_scale_snap, gizmo_scale_snap};
                    else
                        snap_values = {gizmo_translate_snap, gizmo_translate_snap, gizmo_translate_snap};

                    if (ImGuizmo::Manipulate(view.data(),
                                             projection.data(),
                                             gizmo_operation,
                                             gizmo_mode,
                                             matrix.data(),
                                             nullptr,
                                             gizmo_snap_enabled ? snap_values.data() : nullptr))
                    {
                        if (gizmo_operation == ImGuizmo::TRANSLATE)
                        {
                            map.set_mesh_origin(*selected_mesh, omath::mat_extract_origin(omath::opengl_engine::Mat4X4(matrix.data())));
                        }
                        else
                            map.set_mesh_matrix(*selected_mesh, omath::opengl_engine::Mat4X4(matrix.data()));

                        world.update_collider(
                            *selected_mesh,
                            CollisionWorld::Collider(map.get_meshes()[*selected_mesh].cpu_mesh()));
                    }
                }
            }

            const bool left_mouse_pressed = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            if (!mouse_captured
                && left_mouse_pressed
                && !left_mouse_was_pressed
                && (!ImGui::GetIO().WantCaptureMouse || (scene_gizmo_window_hovered && !overlay_window_hovered))
                && !ImGuizmo::IsOver()
                && !ImGuizmo::IsUsing())
            {
                double cursor_x = 0.0;
                double cursor_y = 0.0;
                glfwGetCursorPos(m_window, &cursor_x, &cursor_y);

                int window_width = 1;
                int window_height = 1;
                glfwGetWindowSize(m_window, &window_width, &window_height);

                const float scale_x = window_width > 0
                    ? static_cast<float>(framebuffer.x) / static_cast<float>(window_width)
                    : 1.f;
                const float scale_y = window_height > 0
                    ? static_cast<float>(framebuffer.y) / static_cast<float>(window_height)
                    : 1.f;

                selected_mesh = map.pick_mesh(
                    {static_cast<float>(cursor_x) * scale_x, static_cast<float>(cursor_y) * scale_y},
                    camera);
            }
            left_mouse_was_pressed = left_mouse_pressed;

            ImGui::Render();

            if (m_renderer->begin_frame())
            {
                map.draw(*m_renderer, camera, selected_mesh);
                m_renderer->render_imgui(ImGui::GetDrawData());

                const bool capture_frame = plugin->is_ready_to_stream();
                auto bmp_data = m_renderer->end_frame(capture_frame);
                if (capture_frame && !bmp_data.empty())
                    plugin->push_frame(bmp_data);
            }

            framebuffer = m_renderer->framebuffer_size();
            camera.set_view_port({static_cast<float>(framebuffer.x), static_cast<float>(framebuffer.y)});

            if (fps_cap_enabled)
            {
                fps_limit = std::clamp(fps_limit, 1, 1000);
                const double frame_target = current_time + 1.0 / static_cast<double>(fps_limit);
                const double sleep_s = frame_target - 0.002 - glfwGetTime();
                if (sleep_s > 0.0)
                    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
                while (glfwGetTime() < frame_target)
                    ;
            }
        }

        m_renderer->wait_idle();
    }
} // namespace rose::core
