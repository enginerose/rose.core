//
// Created by orange on 25.02.2026.
//
#pragma once
#include "rose/core/vulkan/texture.hpp"
#include <array>
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

    struct PbrMaterial final
    {
        std::array<float, 4> base_color_factor{1.0f, 1.0f, 1.0f, 1.0f};
        std::array<float, 3> emissive_factor{0.0f, 0.0f, 0.0f};
        float metallic_factor = 0.0f;
        float roughness_factor = 0.8f;
        float normal_scale = 1.0f;
    };

    class Mesh final
    {
    public:
        Mesh() = delete;

        Mesh(omath::opengl_engine::Mesh cpu_mesh,
             std::vector<MeshTexture> textures,
             PbrMaterial material = {})
            : m_cpu_mesh(std::move(cpu_mesh))
            , m_textures(std::move(textures))
            , m_material(material)
        {}

        [[nodiscard]] omath::opengl_engine::Mesh& cpu_mesh() noexcept { return m_cpu_mesh; }
        [[nodiscard]] const omath::opengl_engine::Mesh& cpu_mesh() const noexcept { return m_cpu_mesh; }
        [[nodiscard]] const std::vector<MeshTexture>& textures() const noexcept { return m_textures; }
        [[nodiscard]] const PbrMaterial& material() const noexcept { return m_material; }

    private:
        omath::opengl_engine::Mesh m_cpu_mesh;
        std::vector<MeshTexture> m_textures;
        PbrMaterial m_material;
    };
} // namespace rose::core::vulkan
