//
// Created by orange on 25.02.2026.
//
#pragma once
#include <filesystem>
#include <vector>
#include "rose/core/opengl/mesh.hpp"

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

        // Draw all meshes. Caller must call shader.use() before this.
        void draw(const opengl::ShaderProgram& shader) const;

    private:
        std::vector<opengl::Mesh> m_meshes;

        void load(const std::filesystem::path& path);
    };
} // namespace rose::core
