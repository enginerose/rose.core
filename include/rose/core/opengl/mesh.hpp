//
// Created by orange on 25.02.2026.
//
#pragma once
#include <GL/glew.h>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>
#include "rose/core/opengl/texture.hpp"
#include "rose/core/opengl/shader_program.hpp"

namespace rose::core::opengl
{
    struct Vertex
    {
        float position[3];
        float normal[3];
        float uv[2];
    };

    struct MeshTexture
    {
        std::shared_ptr<Texture> texture;
        TextureType type;
    };

    class Mesh final
    {
    public:
        Mesh() = default;

        Mesh(std::vector<Vertex> vertices, std::vector<uint32_t> indices, std::vector<MeshTexture> textures)
            : m_textures(std::move(textures))
        {
            setup(vertices, indices);
        }

        ~Mesh() { destroy(); }

        Mesh(const Mesh&) = delete;
        Mesh& operator=(const Mesh&) = delete;

        Mesh(Mesh&& other) noexcept
            : m_vao(std::exchange(other.m_vao, 0))
            , m_vbo(std::exchange(other.m_vbo, 0))
            , m_ebo(std::exchange(other.m_ebo, 0))
            , m_index_count(std::exchange(other.m_index_count, 0u))
            , m_textures(std::move(other.m_textures))
        {}

        Mesh& operator=(Mesh&& other) noexcept
        {
            if (this != &other)
            {
                destroy();
                m_vao         = std::exchange(other.m_vao, 0);
                m_vbo         = std::exchange(other.m_vbo, 0);
                m_ebo         = std::exchange(other.m_ebo, 0);
                m_index_count = std::exchange(other.m_index_count, 0u);
                m_textures    = std::move(other.m_textures);
            }
            return *this;
        }

        // Bind textures, set uniforms, and draw. The caller must call shader.use() first.
        void draw(const ShaderProgram& shader) const
        {
            bool has_texture = false;
            for (const auto& mt : m_textures)
            {
                if (mt.type == TextureType::BaseColor && mt.texture && mt.texture->valid())
                {
                    mt.texture->bind(0);
                    shader.set_int("uBaseColor", 0);
                    has_texture = true;
                    break;
                }
            }
            shader.set_bool("uHasTexture", has_texture);

            glBindVertexArray(m_vao);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_index_count), GL_UNSIGNED_INT, nullptr);
            glBindVertexArray(0);
        }

    private:
        GLuint   m_vao         = 0;
        GLuint   m_vbo         = 0;
        GLuint   m_ebo         = 0;
        uint32_t m_index_count = 0;
        std::vector<MeshTexture> m_textures;

        void setup(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices)
        {
            glGenVertexArrays(1, &m_vao);
            glGenBuffers(1, &m_vbo);
            glGenBuffers(1, &m_ebo);

            glBindVertexArray(m_vao);

            glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
            glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                         vertices.data(),
                         GL_STATIC_DRAW);

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)),
                         indices.data(),
                         GL_STATIC_DRAW);

            // position: location 0, vec3
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                  reinterpret_cast<void*>(offsetof(Vertex, position)));
            glEnableVertexAttribArray(0);

            // normal: location 1, vec3
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                  reinterpret_cast<void*>(offsetof(Vertex, normal)));
            glEnableVertexAttribArray(1);

            // uv: location 2, vec2
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                  reinterpret_cast<void*>(offsetof(Vertex, uv)));
            glEnableVertexAttribArray(2);

            glBindVertexArray(0);

            m_index_count = static_cast<uint32_t>(indices.size());
        }

        void destroy() noexcept
        {
            if (m_vao != 0) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
            if (m_vbo != 0) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
            if (m_ebo != 0) { glDeleteBuffers(1, &m_ebo); m_ebo = 0; }
        }
    };
} // namespace rose::core::opengl
