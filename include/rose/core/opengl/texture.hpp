//
// Created by orange on 25.02.2026.
//
#pragma once
#include <GL/glew.h>
#include <utility>

namespace rose::core::opengl
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
        {
            GLenum fmt = GL_RGBA;
            switch (components)
            {
            case 1: fmt = GL_RED;  break;
            case 2: fmt = GL_RG;   break;
            case 3: fmt = GL_RGB;  break;
            case 4: fmt = GL_RGBA; break;
            default: break;
            }

            glGenTextures(1, &m_id);
            glBindTexture(GL_TEXTURE_2D, m_id);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(fmt), width, height, 0, fmt, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, 0);
        }

        ~Texture() { destroy(); }

        Texture(const Texture&) = delete;
        Texture& operator=(const Texture&) = delete;

        Texture(Texture&& other) noexcept
            : m_id(std::exchange(other.m_id, 0))
        {}

        Texture& operator=(Texture&& other) noexcept
        {
            if (this != &other)
            {
                destroy();
                m_id = std::exchange(other.m_id, 0);
            }
            return *this;
        }

        void bind(unsigned int unit = 0) const
        {
            glActiveTexture(GL_TEXTURE0 + unit);
            glBindTexture(GL_TEXTURE_2D, m_id);
        }

        GLuint id() const noexcept { return m_id; }
        bool valid() const noexcept { return m_id != 0; }

    private:
        GLuint m_id = 0;

        void destroy() noexcept
        {
            if (m_id != 0)
            {
                glDeleteTextures(1, &m_id);
                m_id = 0;
            }
        }
    };
} // namespace rose::core::opengl
