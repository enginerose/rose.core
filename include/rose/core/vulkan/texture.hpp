//
// Created by orange on 25.02.2026.
//
#pragma once
#include <vector>

namespace rose::core::vulkan
{
    enum class TextureType
    {
        BaseColor,
        Normal,
        MetallicRoughness,
        Emissive,
    };

    class Texture final
    {
    public:
        Texture() = default;

        Texture(int width, int height, int components, const unsigned char* data)
            : m_width(width)
            , m_height(height)
            , m_components(components)
        {
            if (data != nullptr && width > 0 && height > 0 && components > 0)
            {
                const auto size = static_cast<std::size_t>(width)
                                * static_cast<std::size_t>(height)
                                * static_cast<std::size_t>(components);
                m_pixels.assign(data, data + size);
            }
        }

        [[nodiscard]] int width() const noexcept { return m_width; }
        [[nodiscard]] int height() const noexcept { return m_height; }
        [[nodiscard]] int components() const noexcept { return m_components; }
        [[nodiscard]] const std::vector<unsigned char>& pixels() const noexcept { return m_pixels; }
        [[nodiscard]] bool valid() const noexcept { return !m_pixels.empty() && m_width > 0 && m_height > 0; }

    private:
        int m_width      = 0;
        int m_height     = 0;
        int m_components = 0;
        std::vector<unsigned char> m_pixels;
    };
} // namespace rose::core::vulkan
