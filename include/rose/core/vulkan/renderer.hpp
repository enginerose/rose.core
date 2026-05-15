//
// Created by orange on 15.05.2026.
//
#pragma once
#include "rose/core/vulkan/mesh.hpp"
#include <cstddef>
#include <memory>
#include <omath/engines/opengl_engine/camera.hpp>
#include <omath/linear_algebra/vector2.hpp>
#include <vector>

struct GLFWwindow;
struct ImDrawData;

namespace rose::core::vulkan
{
    class Renderer final
    {
    public:
        Renderer(GLFWwindow* window, const omath::Vector2<int>& initial_size);
        ~Renderer();

        Renderer(const Renderer&) = delete;
        Renderer& operator=(const Renderer&) = delete;

        [[nodiscard]] bool begin_frame();
        void draw_mesh(const Mesh& mesh, const omath::opengl_engine::Camera& camera);
        void render_imgui(ImDrawData* draw_data);
        [[nodiscard]] std::vector<std::byte> end_frame(bool capture_screenshot);
        void wait_idle() const;

        [[nodiscard]] omath::Vector2<int> framebuffer_size() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
} // namespace rose::core::vulkan
