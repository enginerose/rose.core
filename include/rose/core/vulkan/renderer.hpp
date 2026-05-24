//
// Created by orange on 15.05.2026.
//
#pragma once
#include "rose/core/vulkan/mesh.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <omath/engines/opengl_engine/camera.hpp>
#include <omath/linear_algebra/vector2.hpp>
#include <optional>
#include <string>
#include <vector>

struct GLFWwindow;
struct ImDrawData;

namespace rose::core::vulkan
{
    enum class DlssQuality : int
    {
        Quality,
        Balanced,
        Performance,
        UltraPerformance,
        UltraQuality,
        Dlaa
    };

    struct SelectionOutlineSettings final
    {
        std::array<float, 3> color{0.02f, 0.72f, 1.0f};
        float width = 0.18f;
        int smoothing_quality = 3;
    };

    struct BloomSettings final
    {
        bool enabled = false;
        float threshold = 1.0f;
        float intensity = 0.35f;
        float radius = 2.0f;
        int quality = 8;
    };

    enum class CapturedFrameFormat
    {
        Rgba,
        Bgra
    };

    struct CapturedFrame final
    {
        std::vector<std::byte> pixels;
        uint32_t width = 0;
        uint32_t height = 0;
        CapturedFrameFormat format = CapturedFrameFormat::Rgba;
    };

    class Renderer final
    {
    public:
        Renderer(GLFWwindow* window, const omath::Vector2<int>& initial_size);
        ~Renderer();

        Renderer(const Renderer&) = delete;
        Renderer& operator=(const Renderer&) = delete;

        [[nodiscard]] bool begin_frame();
        void draw_mesh(const Mesh& mesh, const omath::opengl_engine::Camera& camera);
        void draw_mesh_outline(const Mesh& mesh, const omath::opengl_engine::Camera& camera);
        void render_imgui(ImDrawData* draw_data);
        [[nodiscard]] std::optional<CapturedFrame> end_frame(bool capture_screenshot);
        void wait_idle() const;

        [[nodiscard]] omath::Vector2<int> framebuffer_size() const;
        [[nodiscard]] bool dlss_available() const;
        [[nodiscard]] bool dlss_enabled() const;
        void set_dlss_enabled(bool enabled);
        [[nodiscard]] DlssQuality dlss_quality() const;
        void set_dlss_quality(DlssQuality quality);
        [[nodiscard]] std::string dlss_status() const;
        [[nodiscard]] SelectionOutlineSettings selection_outline_settings() const;
        void set_selection_outline_settings(const SelectionOutlineSettings& settings);
        [[nodiscard]] BloomSettings bloom_settings() const;
        void set_bloom_settings(const BloomSettings& settings);

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
} // namespace rose::core::vulkan
