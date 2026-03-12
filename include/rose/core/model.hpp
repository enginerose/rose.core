//
// Created by orange on 25.02.2026.
//
#pragma once
#include "rose/core/collision_world.hpp"
#include "rose/core/opengl/mesh.hpp"
#include "rose/core/opengl/shader_program.hpp"
#include <omath/engines/opengl_engine/camera.hpp>
#include <omath/engines/opengl_engine/mesh.hpp>
#include <filesystem>
#include <vector>

namespace rose::core
{
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

        [[nodiscard]] const std::vector<opengl::Mesh>& get_meshes() const { return m_meshes; }

        // Draw visible meshes. Caller must call shader.use() before this.
        void draw(const opengl::ShaderProgram& shader,
                  const omath::opengl_engine::Camera& camera) const;

    private:
        std::vector<opengl::Mesh> m_meshes;
        std::vector<Aabb>         m_mesh_aabbs; // world-space AABB per mesh, parallel to m_meshes

        void load(const std::filesystem::path& path);
    };
} // namespace rose::core
