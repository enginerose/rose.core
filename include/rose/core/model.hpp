//
// Created by orange on 25.02.2026.
//
#pragma once
#include "rose/core/collision_world.hpp"
#include "rose/core/vulkan/mesh.hpp"
#include <omath/engines/opengl_engine/camera.hpp>
#include <omath/engines/opengl_engine/mesh.hpp>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <vector>

namespace rose::core
{
    namespace vulkan
    {
        class Renderer;
    }

    class Model final
    {
    public:
        explicit Model(const std::filesystem::path& path);

        Model(const Model&) = delete;
        Model& operator=(const Model&) = delete;
        Model(Model&&) = default;
        Model& operator=(Model&&) = default;

        void set_rotation(const omath::opengl_engine::ViewAngles& angles)
        {
            for (auto& mesh : m_meshes)
                mesh.cpu_mesh().set_rotation(angles);
        }

        [[nodiscard]] const omath::opengl_engine::ViewAngles& get_rotation() const
        {
            return m_meshes.front().cpu_mesh().get_rotation_angles();
        }

        [[nodiscard]] const std::vector<vulkan::Mesh>& get_meshes() const { return m_meshes; }
        [[nodiscard]] std::optional<omath::opengl_engine::Mat4X4> mesh_matrix(std::size_t mesh_index) const;
        void set_mesh_origin(std::size_t mesh_index, const omath::Vector3<float>& origin);
        void set_mesh_matrix(std::size_t mesh_index, const omath::opengl_engine::Mat4X4& matrix);
        [[nodiscard]] std::optional<std::size_t> pick_mesh(const omath::Vector2<float>& screen_position,
                                                           const omath::opengl_engine::Camera& camera) const;

        void draw(vulkan::Renderer& renderer,
                  const omath::opengl_engine::Camera& camera,
                  std::optional<std::size_t> selected_mesh = std::nullopt) const;

    private:
        std::vector<vulkan::Mesh> m_meshes;
        std::vector<Aabb>         m_mesh_aabbs; // world-space AABB per mesh, parallel to m_meshes

        void load(const std::filesystem::path& path);
    };
} // namespace rose::core
