//
// Created by orange on 25.02.2026.
//
#pragma once
#include "rose/core/vulkan/texture.hpp"
#include <memory>
#include <omath/engines/opengl_engine/mesh.hpp>
#include <utility>
#include <vector>

namespace rose::core::vulkan
{
    struct MeshTexture
    {
        std::shared_ptr<Texture> texture;
        TextureType type;
    };

    class Mesh final
    {
    public:
        Mesh() = delete;

        Mesh(omath::opengl_engine::Mesh cpu_mesh, std::vector<MeshTexture> textures)
            : m_cpu_mesh(std::move(cpu_mesh))
            , m_textures(std::move(textures))
        {}

        [[nodiscard]] omath::opengl_engine::Mesh& cpu_mesh() noexcept { return m_cpu_mesh; }
        [[nodiscard]] const omath::opengl_engine::Mesh& cpu_mesh() const noexcept { return m_cpu_mesh; }
        [[nodiscard]] const std::vector<MeshTexture>& textures() const noexcept { return m_textures; }

    private:
        omath::opengl_engine::Mesh m_cpu_mesh;
        std::vector<MeshTexture> m_textures;
    };
} // namespace rose::core::vulkan
