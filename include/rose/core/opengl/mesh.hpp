//
// Created by orange on 25.02.2026.
//
#pragma once
#include <GL/glew.h>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>
#include <omath/engines/opengl_engine/mesh.hpp>
#include "rose/core/opengl/texture.hpp"
#include "rose/core/opengl/shader_program.hpp"

namespace rose::core::opengl
{
    struct MeshTexture
    {
        std::shared_ptr<Texture> texture;
        TextureType type;
    };

    class Mesh final
    {
    public:
        Mesh() = default;

        Mesh(omath::opengl_engine::Mesh cpu_mesh, std::vector<MeshTexture> textures)
            : m_cpu_mesh(std::move(cpu_mesh))
            , m_textures(std::move(textures))
        {
            static_assert(sizeof(omath::Vector3<uint32_t>) == 3 * sizeof(uint32_t),
                          "omath::Vector3 must be tightly packed");
            setup();
        }

        ~Mesh() { destroy(); }

        Mesh(const Mesh&) = delete;
        Mesh& operator=(const Mesh&) = delete;

        Mesh(Mesh&& other) noexcept
            : m_vao(std::exchange(other.m_vao, 0))
            , m_vbo(std::exchange(other.m_vbo, 0))
            , m_ebo(std::exchange(other.m_ebo, 0))
            , m_index_count(std::exchange(other.m_index_count, 0u))
            , m_cpu_mesh(std::move(other.m_cpu_mesh))
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
                m_cpu_mesh    = std::move(other.m_cpu_mesh);
                m_textures    = std::move(other.m_textures);
            }
            return *this;
        }

        omath::opengl_engine::Mesh& cpu_mesh() { return m_cpu_mesh; }
        const omath::opengl_engine::Mesh& cpu_mesh() const { return m_cpu_mesh; }

        // Bind textures, set uModel, and draw. Caller must call shader.use() first.
        void draw(const ShaderProgram& shader) const
        {
            shader.set_mat4("uModel", m_cpu_mesh.get_to_world_matrix().raw_array().data());

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
        omath::opengl_engine::Mesh m_cpu_mesh;
        std::vector<MeshTexture> m_textures;

        void setup()
        {
            const auto& vertices  = m_cpu_mesh.m_vertex_buffer;
            const auto& triangles = m_cpu_mesh.m_element_buffer_object;

            glGenVertexArrays(1, &m_vao);
            glGenBuffers(1, &m_vbo);
            glGenBuffers(1, &m_ebo);

            glBindVertexArray(m_vao);

            using VertexType = omath::opengl_engine::Mesh::VertexType;
            constexpr auto stride = static_cast<GLsizei>(sizeof(VertexType));
            constexpr auto vec3sz = static_cast<GLsizeiptr>(sizeof(omath::Vector3<float>));

            glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
            glBufferData(GL_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexType)),
                         vertices.data(),
                         GL_STATIC_DRAW);

            // Each Vector3<uint32_t> stores one triangle as 3 packed indices.
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                         static_cast<GLsizeiptr>(triangles.size() * sizeof(omath::Vector3<uint32_t>)),
                         triangles.data(),
                         GL_STATIC_DRAW);

            // location 0: position (Vector3<float> at offset 0)
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
            glEnableVertexAttribArray(0);

            // location 1: normal (Vector3<float> at offset 12)
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(vec3sz));
            glEnableVertexAttribArray(1);

            // location 2: uv (Vector2<float> at offset 24)
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(2 * vec3sz));
            glEnableVertexAttribArray(2);

            glBindVertexArray(0);

            m_index_count = static_cast<uint32_t>(triangles.size() * 3);
        }

        void destroy() noexcept
        {
            if (m_vao != 0) { glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
            if (m_vbo != 0) { glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
            if (m_ebo != 0) { glDeleteBuffers(1, &m_ebo); m_ebo = 0; }
        }
    };
} // namespace rose::core::opengl
