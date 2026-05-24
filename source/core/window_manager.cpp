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

#include "stb_image_write.h"

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
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

static void GlfwErrorCallback(int code, const char* desc)
{
    spdlog::error("GLFW error {}: {}", code, desc);
}

static std::vector<std::byte> EncodeStreamFrame(const rose::core::vulkan::CapturedFrame& frame)
{
    if (frame.pixels.empty() || frame.width == 0 || frame.height == 0)
        return {};

    const auto* source = reinterpret_cast<const unsigned char*>(frame.pixels.data());
    const unsigned char* pixels = source;
    std::vector<unsigned char> rgba;
    if (frame.format == rose::core::vulkan::CapturedFrameFormat::Bgra)
    {
        rgba.resize(frame.pixels.size());
        for (std::size_t index = 0; index < frame.pixels.size(); index += 4u)
        {
            rgba[index] = source[index + 2u];
            rgba[index + 1u] = source[index + 1u];
            rgba[index + 2u] = source[index];
            rgba[index + 3u] = source[index + 3u];
        }
        pixels = rgba.data();
    }

    std::vector<std::byte> bmp;
    stbi_write_bmp_to_func(
        [](void* context, void* data, int size)
        {
            auto* output = static_cast<std::vector<std::byte>*>(context);
            const auto* bytes = static_cast<std::byte*>(data);
            output->insert(output->end(), bytes, bytes + size);
        },
        &bmp,
        static_cast<int>(frame.width),
        static_cast<int>(frame.height),
        4,
        pixels);
    return bmp;
}

static rose::core::vulkan::Mesh CreateMarkerMesh(const std::array<float, 4>& base_color,
                                                 const std::array<float, 3>& emissive,
                                                 const omath::Vector3<float>& origin,
                                                 const omath::Vector3<float>& scale)
{
    using Vertex = omath::opengl_engine::Mesh::VertexType;

    std::vector<Vertex> vertices{
        {{-0.5f, -0.5f, -0.5f}, {-1.0f, -1.0f, -1.0f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f, -0.5f}, { 1.0f, -1.0f, -1.0f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f, -0.5f}, { 1.0f,  1.0f, -1.0f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f, -0.5f}, {-1.0f,  1.0f, -1.0f}, {0.0f, 1.0f}},
        {{-0.5f, -0.5f,  0.5f}, {-1.0f, -1.0f,  1.0f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f,  0.5f}, { 1.0f, -1.0f,  1.0f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f,  0.5f}, { 1.0f,  1.0f,  1.0f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f,  0.5f}, {-1.0f,  1.0f,  1.0f}, {0.0f, 1.0f}},
    };
    std::vector<omath::Vector3<uint32_t>> triangles{
        {0, 2, 1}, {0, 3, 2},
        {4, 5, 6}, {4, 6, 7},
        {0, 1, 5}, {0, 5, 4},
        {3, 6, 2}, {3, 7, 6},
        {1, 2, 6}, {1, 6, 5},
        {0, 4, 7}, {0, 7, 3},
    };

    rose::core::vulkan::PbrMaterial material;
    material.base_color_factor = base_color;
    material.emissive_factor = emissive;

    rose::core::vulkan::Mesh mesh{
        omath::opengl_engine::Mesh{std::move(vertices), std::move(triangles)},
        {},
        material
    };
    mesh.cpu_mesh().set_origin(origin);
    mesh.cpu_mesh().set_scale(scale);
    return mesh;
}

static rose::core::vulkan::Mesh CreateSpotlightMarkerMesh()
{
    return CreateMarkerMesh({1.0f, 0.25f, 0.75f, 1.0f},
                            {1.0f, 0.25f, 0.75f},
                            {0.0f, 5.0f, 0.0f},
                            {0.25f, 0.25f, 0.25f});
}

static rose::core::vulkan::Mesh CreateSunMarkerMesh()
{
    return CreateMarkerMesh({1.0f, 0.86f, 0.18f, 1.0f},
                            {1.0f, 0.62f, 0.08f},
                            {8.0f, 16.0f, 8.0f},
                            {0.45f, 0.45f, 0.45f});
}

static omath::Vector3<float> Normalized(omath::Vector3<float> value)
{
    const float length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    if (length <= 0.0001f)
        return {0.0f, 0.0f, -1.0f};
    return value / length;
}

static omath::Vector3<float> MatrixForward(const omath::opengl_engine::Mat4X4& matrix)
{
    return Normalized({
        -matrix.at(0, 2),
        -matrix.at(1, 2),
        -matrix.at(2, 2)
    });
}

static bool PickProjectedSphere(const omath::Vector2<float>& screen_position,
                                const omath::opengl_engine::Camera& camera,
                                const omath::Vector3<float>& center,
                                const omath::Vector2<int>& framebuffer_size,
                                float pixel_radius)
{
    const auto& view_projection = camera.get_view_projection_matrix();
    const float clip_x = view_projection.at(0, 0) * center.x
                       + view_projection.at(0, 1) * center.y
                       + view_projection.at(0, 2) * center.z
                       + view_projection.at(0, 3);
    const float clip_y = view_projection.at(1, 0) * center.x
                       + view_projection.at(1, 1) * center.y
                       + view_projection.at(1, 2) * center.z
                       + view_projection.at(1, 3);
    const float clip_w = view_projection.at(3, 0) * center.x
                       + view_projection.at(3, 1) * center.y
                       + view_projection.at(3, 2) * center.z
                       + view_projection.at(3, 3);
    if (clip_w <= 0.0001f)
        return false;

    const float ndc_x = clip_x / clip_w;
    const float ndc_y = clip_y / clip_w;
    const float screen_x = (ndc_x * 0.5f + 0.5f) * static_cast<float>(framebuffer_size.x);
    const float screen_y = (0.5f - ndc_y * 0.5f) * static_cast<float>(framebuffer_size.y);
    const float dx = screen_position.x - screen_x;
    const float dy = screen_position.y - screen_y;
    return dx * dx + dy * dy <= pixel_radius * pixel_radius;
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
        std::shared_ptr<StreamPluginApi> plugin;
        try
        {
            auto creator = boost::dll::import_alias<pluginapi_create_t>(
                lib_path,
                "create_plugin",
                boost::dll::load_mode::default_mode
            );
            plugin = creator();
            if (plugin != nullptr)
                plugin->run();
        }
        catch (const std::exception& exception)
        {
            spdlog::warn("Frame streaming disabled: {}", exception.what());
        }

        std::mutex stream_mutex;
        std::condition_variable stream_condition;
        std::optional<vulkan::CapturedFrame> pending_stream_frame;
        std::atomic_bool stream_ready = false;
        std::atomic_bool stream_worker_busy = false;
        bool stop_stream_worker = false;
        std::thread stream_worker;
        if (plugin != nullptr)
        {
            stream_worker = std::thread(
                [plugin,
                 &stream_mutex,
                 &stream_condition,
                 &pending_stream_frame,
                 &stream_ready,
                 &stream_worker_busy,
                 &stop_stream_worker]
                {
                    bool poll_error_logged = false;
                    bool push_error_logged = false;
                    while (true)
                    {
                        std::optional<vulkan::CapturedFrame> frame;
                        {
                            std::unique_lock lock(stream_mutex);
                            stream_condition.wait_for(lock, std::chrono::milliseconds(100), [&]
                            {
                                return stop_stream_worker || pending_stream_frame.has_value();
                            });
                            if (stop_stream_worker)
                                break;
                            if (!pending_stream_frame)
                            {
                                lock.unlock();
                                try
                                {
                                    stream_ready.store(plugin->is_ready_to_stream(), std::memory_order_release);
                                }
                                catch (const std::exception& exception)
                                {
                                    stream_ready.store(false, std::memory_order_release);
                                    if (!poll_error_logged)
                                    {
                                        spdlog::warn("Frame streaming readiness check failed: {}", exception.what());
                                        poll_error_logged = true;
                                    }
                                }
                                continue;
                            }
                            frame = std::move(pending_stream_frame);
                            pending_stream_frame.reset();
                        }

                        if (!frame)
                            continue;

                        stream_worker_busy.store(true, std::memory_order_release);
                        try
                        {
                            std::vector<std::byte> bmp = EncodeStreamFrame(*frame);
                            if (!bmp.empty())
                                plugin->push_frame(bmp);
                            stream_ready.store(true, std::memory_order_release);
                        }
                        catch (const std::exception& exception)
                        {
                            if (!push_error_logged)
                            {
                                spdlog::warn("Frame streaming push failed: {}", exception.what());
                                push_error_logged = true;
                            }
                        }
                        stream_worker_busy.store(false, std::memory_order_release);
                    }
                });
        }

        const auto stream_has_pending_frame = [&]
        {
            std::lock_guard lock(stream_mutex);
            return pending_stream_frame.has_value();
        };

        const auto queue_stream_frame = [&](vulkan::CapturedFrame frame)
        {
            {
                std::lock_guard lock(stream_mutex);
                pending_stream_frame = std::move(frame);
            }
            stream_condition.notify_one();
        };

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
        auto spotlight_marker = CreateSpotlightMarkerMesh();
        auto spotlight_settings = m_renderer->spotlight_settings();
        auto spotlight_direction = spotlight_settings.direction;
        spotlight_marker.cpu_mesh().set_origin(spotlight_settings.position);
        auto sun_marker = CreateSunMarkerMesh();
        auto sun_settings = m_renderer->sun_settings();
        sun_marker.cpu_mesh().set_origin(Normalized(-sun_settings.direction) * 18.0f);

        constexpr float radians_per_degree = 3.14159265358979323846f / 180.0f;
        constexpr float degrees_per_radian = 180.0f / 3.14159265358979323846f;
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
        bool   wallrun_enabled = false;
        float  ground_max_slope_degrees = std::acos(Player::k_floor_dot) * degrees_per_radian;
        int    fps_limit = 60;
        std::optional<std::size_t> selected_mesh;
        bool   spotlight_selected = false;
        bool   sun_selected = false;
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
        double next_stream_capture_time = 0.0;
        constexpr double stream_capture_interval = 1.0 / 30.0;

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
            if ((selected_mesh || spotlight_selected || sun_selected) && !mouse_captured)
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
                if (ImGui::BeginTabBar("ROSE Overlay Tabs"))
                {
                    if (ImGui::BeginTabItem("General"))
                    {
                        ImGui::Checkbox("FPS cap", &fps_cap_enabled);
                        ImGui::BeginDisabled(!fps_cap_enabled);
                        ImGui::SliderInt("FPS limit", &fps_limit, 30, 360);
                        ImGui::EndDisabled();
                        ImGui::Checkbox("Auto bhop", &auto_bhop);
                        ImGui::Checkbox("Wallrun", &wallrun_enabled);
                        if (ImGui::SliderFloat("Ground max slope", &ground_max_slope_degrees, 0.0f, 89.0f, "%.1f deg"))
                        {
                            player.set_floor_dot(std::cos(ground_max_slope_degrees * radians_per_degree));
                        }
                        ImGui::EndTabItem();
                    }

                    if (ImGui::BeginTabItem("Rendering"))
                    {
                        auto bloom_settings = m_renderer->bloom_settings();
                        bool bloom_settings_changed = false;
                        bloom_settings_changed |= ImGui::Checkbox("Bloom", &bloom_settings.enabled);
                        ImGui::BeginDisabled(!bloom_settings.enabled);
                        bloom_settings_changed |= ImGui::SliderFloat("Bloom threshold",
                                                                     &bloom_settings.threshold,
                                                                     0.0f,
                                                                     4.0f,
                                                                     "%.2f");
                        bloom_settings_changed |= ImGui::SliderFloat("Bloom intensity",
                                                                     &bloom_settings.intensity,
                                                                     0.0f,
                                                                     3.0f,
                                                                     "%.2f");
                        bloom_settings_changed |= ImGui::SliderFloat("Bloom radius",
                                                                     &bloom_settings.radius,
                                                                     0.0f,
                                                                     16.0f,
                                                                     "%.1f px");
                        bloom_settings_changed |= ImGui::SliderInt("Bloom quality",
                                                                   &bloom_settings.quality,
                                                                   1,
                                                                   64);
                        ImGui::EndDisabled();
                        if (bloom_settings_changed)
                            m_renderer->set_bloom_settings(bloom_settings);

                        ImGui::Separator();
                        bool spotlight_changed = false;
                        spotlight_changed |= ImGui::Checkbox("Spotlight", &spotlight_settings.enabled);
                        ImGui::BeginDisabled(!spotlight_settings.enabled);
                        spotlight_changed |= ImGui::ColorEdit3("Light color", spotlight_settings.color.data());
                        spotlight_changed |= ImGui::SliderFloat("Light intensity",
                                                                &spotlight_settings.intensity,
                                                                0.0f,
                                                                250.0f,
                                                                "%.1f");
                        spotlight_changed |= ImGui::SliderFloat("Light range",
                                                                &spotlight_settings.range,
                                                                1.0f,
                                                                100.0f,
                                                                "%.1f m");
                        spotlight_changed |= ImGui::SliderFloat("Inner cone",
                                                                &spotlight_settings.inner_angle_degrees,
                                                                1.0f,
                                                                89.0f,
                                                                "%.1f deg");
                        spotlight_changed |= ImGui::SliderFloat("Outer cone",
                                                                &spotlight_settings.outer_angle_degrees,
                                                                1.0f,
                                                                89.9f,
                                                                "%.1f deg");
                        ImGui::EndDisabled();
                        if (spotlight_changed && spotlight_settings.outer_angle_degrees <= spotlight_settings.inner_angle_degrees)
                            spotlight_settings.outer_angle_degrees = spotlight_settings.inner_angle_degrees + 0.1f;

                        ImGui::Separator();
                        bool sun_changed = false;
                        sun_changed |= ImGui::Checkbox("Sun", &sun_settings.enabled);
                        ImGui::BeginDisabled(!sun_settings.enabled);
                        sun_changed |= ImGui::ColorEdit3("Sun color", sun_settings.color.data());
                        sun_changed |= ImGui::SliderFloat("Sun intensity",
                                                          &sun_settings.intensity,
                                                          0.0f,
                                                          10.0f,
                                                          "%.2f");
                        sun_changed |= ImGui::SliderFloat("Sun shadow area",
                                                          &sun_settings.shadow_distance,
                                                          5.0f,
                                                          200.0f,
                                                          "%.1f m");
                        ImGui::EndDisabled();
                        if (sun_changed)
                            m_renderer->set_sun_settings(sun_settings);

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
                        ImGui::EndTabItem();
                    }

                    if (ImGui::BeginTabItem("Selection"))
                    {
                        if (selected_mesh)
                            ImGui::Text("Selected mesh: %zu", *selected_mesh);
                        else if (spotlight_selected)
                            ImGui::TextUnformatted("Selected object: spotlight");
                        else if (sun_selected)
                            ImGui::TextUnformatted("Selected object: sun");
                        else
                            ImGui::TextUnformatted("Selected object: none");
                        ImGui::BeginDisabled(!selected_mesh && !spotlight_selected && !sun_selected);
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
                        ImGui::EndTabItem();
                    }

                    ImGui::EndTabBar();
                }

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
                && (selected_mesh || spotlight_selected || sun_selected)
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
                input.wallrun  = wallrun_enabled;
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

            if ((selected_mesh || spotlight_selected || sun_selected) && !mouse_captured && scene_gizmo_draw_list != nullptr)
            {
                std::optional<omath::opengl_engine::Mat4X4> model_matrix;
                if (selected_mesh)
                    model_matrix = map.mesh_matrix(*selected_mesh);
                else if (spotlight_selected)
                    model_matrix = spotlight_marker.cpu_mesh().get_to_world_matrix();
                else if (sun_selected)
                    model_matrix = sun_marker.cpu_mesh().get_to_world_matrix();

                if (model_matrix)
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
                        const omath::opengl_engine::Mat4X4 updated_matrix(matrix.data());
                        if (selected_mesh)
                        {
                            if (gizmo_operation == ImGuizmo::TRANSLATE)
                                map.set_mesh_origin(*selected_mesh, omath::mat_extract_origin(updated_matrix));
                            else
                                map.set_mesh_matrix(*selected_mesh, updated_matrix);

                            world.update_collider(
                                *selected_mesh,
                                CollisionWorld::Collider(map.get_meshes()[*selected_mesh].cpu_mesh()));
                        }
                        else if (spotlight_selected)
                        {
                            auto& light_mesh = spotlight_marker.cpu_mesh();
                            light_mesh.set_origin(omath::mat_extract_origin(updated_matrix));
                            if (gizmo_operation == ImGuizmo::ROTATE)
                                spotlight_direction = MatrixForward(updated_matrix);
                            else if (gizmo_operation == ImGuizmo::SCALE)
                                light_mesh.set_scale(omath::mat_extract_scale(updated_matrix));
                        }
                        else if (sun_selected)
                        {
                            auto& light_mesh = sun_marker.cpu_mesh();
                            light_mesh.set_origin(omath::mat_extract_origin(updated_matrix));
                            if (gizmo_operation == ImGuizmo::SCALE)
                                light_mesh.set_scale(omath::mat_extract_scale(updated_matrix));
                        }
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

                const omath::Vector2<float> screen_position{
                    static_cast<float>(cursor_x) * scale_x,
                    static_cast<float>(cursor_y) * scale_y
                };
                if (PickProjectedSphere(screen_position, camera, spotlight_marker.cpu_mesh().get_origin(), framebuffer, 22.0f))
                {
                    spotlight_selected = true;
                    sun_selected = false;
                    selected_mesh.reset();
                }
                else if (PickProjectedSphere(screen_position, camera, sun_marker.cpu_mesh().get_origin(), framebuffer, 26.0f))
                {
                    spotlight_selected = false;
                    sun_selected = true;
                    selected_mesh.reset();
                }
                else
                {
                    selected_mesh = map.pick_mesh(screen_position, camera);
                    spotlight_selected = false;
                    sun_selected = false;
                }
            }
            left_mouse_was_pressed = left_mouse_pressed;

            spotlight_settings.position = spotlight_marker.cpu_mesh().get_origin();
            spotlight_settings.direction = spotlight_direction;
            m_renderer->set_spotlight_settings(spotlight_settings);
            sun_settings.direction = Normalized(-sun_marker.cpu_mesh().get_origin());
            m_renderer->set_sun_settings(sun_settings);

            ImGui::Render();

            if (m_renderer->begin_frame())
            {
                map.draw(*m_renderer, camera, selected_mesh);
                m_renderer->draw_mesh(spotlight_marker, camera);
                if (spotlight_selected)
                    m_renderer->draw_mesh_outline(spotlight_marker, camera);
                m_renderer->draw_mesh(sun_marker, camera);
                if (sun_selected)
                    m_renderer->draw_mesh_outline(sun_marker, camera);
                m_renderer->render_imgui(ImGui::GetDrawData());

                bool capture_frame = false;
                if (plugin != nullptr
                    && current_time >= next_stream_capture_time
                    && stream_ready.load(std::memory_order_acquire)
                    && !stream_worker_busy.load(std::memory_order_acquire)
                    && !stream_has_pending_frame())
                {
                    capture_frame = true;
                    next_stream_capture_time = current_time + stream_capture_interval;
                }
                auto stream_frame = m_renderer->end_frame(capture_frame);
                if (stream_frame && plugin != nullptr)
                    queue_stream_frame(std::move(*stream_frame));
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

        if (stream_worker.joinable())
        {
            {
                std::lock_guard lock(stream_mutex);
                stop_stream_worker = true;
                pending_stream_frame.reset();
            }
            stream_condition.notify_one();
            stream_worker.join();
        }

        m_renderer->wait_idle();
    }
} // namespace rose::core
