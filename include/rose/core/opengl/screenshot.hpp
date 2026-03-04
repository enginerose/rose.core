//
// Created by orange on 04.03.2026.
//
#pragma once
#include <GL/glew.h>
#include <cstddef>
#include <vector>

namespace rose::core
{
    // Reads the current default framebuffer and returns a BMP-encoded image in memory.
    // width/height should match the current viewport dimensions.
    [[nodiscard]] std::vector<std::byte> take_screenshot(int width, int height);
} // namespace rose::core
