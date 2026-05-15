//
// Created by orange on 16.02.2026.
//
#pragma once
#include <memory>
#include <omath/linear_algebra/vector2.hpp>

struct GLFWwindow;

namespace rose::core
{
    namespace vulkan
    {
        class Renderer;
    }

    class WindowManager final
    {
    public:
        explicit WindowManager();
        ~WindowManager();

        WindowManager(const WindowManager&) = delete;
        WindowManager& operator=(const WindowManager&) = delete;

        void run();
    private:
        omath::Vector2<int> m_window_size = {1280, 720};
        GLFWwindow* m_window = nullptr;
        std::unique_ptr<vulkan::Renderer> m_renderer;
    };
} // namespace rose::core
