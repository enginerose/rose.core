//
// Created by orange on 16.02.2026.
//
#pragma once
#include "GLFW/glfw3.h"
#include <cstdint>
#include <omath/linear_algebra/vector2.hpp>

namespace rose::core
{
    class WindowManager final
    {
    public:
        explicit WindowManager();
        void run();
    private:
        omath::Vector2<int> m_window_size = {800, 600};
        GLFWwindow* m_window;
    };
} // namespace rose::core