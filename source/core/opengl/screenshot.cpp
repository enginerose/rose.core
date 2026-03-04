//
// Created by orange on 04.03.2026.
//
#include "rose/core/opengl/screenshot.hpp"
#include "stb_image_write.h"
#include <algorithm>
#include <cstdint>

namespace rose::core
{
    std::vector<std::byte> take_screenshot(int width, int height)
    {
        // Read RGBA pixels from the default framebuffer (bottom-left origin)
        std::vector<std::byte> pixels(static_cast<std::size_t>(width * height * 4));
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

        // Flip rows vertically — OpenGL origin is bottom-left, BMP expects top-left
        // stbi_write_bmp_to_func requires uint8_t* so we use that for the flipped buffer
        const int row_bytes = width * 4;
        std::vector<uint8_t> flipped(pixels.size());
        for (int y = 0; y < height; ++y)
        {
            const std::byte* src = pixels.data() + (height - 1 - y) * row_bytes;
            uint8_t*         dst = flipped.data() + y * row_bytes;
            std::transform(src, src + row_bytes, dst,
                [](std::byte b) { return static_cast<uint8_t>(b); });
        }

        // Encode to BMP in memory via stb_image_write callback
        std::vector<std::byte> bmp;
        stbi_write_bmp_to_func(
            [](void* ctx, void* data, int size)
            {
                auto* out = static_cast<std::vector<std::byte>*>(ctx);
                const auto* bytes = static_cast<std::byte*>(data);
                out->insert(out->end(), bytes, bytes + size);
            },
            &bmp,
            width, height,
            4,           // RGBA channels
            flipped.data()
        );

        return bmp;
    }
} // namespace rose::core
